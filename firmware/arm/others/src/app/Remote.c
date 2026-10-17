#include "Remote.h"
#include "usart.h"
#include <string.h>

#define RC_ONLINE_TIMEOUT_MS  100U
#define RC_CHANNEL_FULL_SCALE 660.0f
#define RC_CHANNEL_DEADBAND   0.02f  /* 归一化死区，扣除后重新缩放 */

/* 上电即视为“遥控丢失”：两个超时判据（IsOnline 100ms / IsLost 3s）都要判为离线，
   这样系统会先挂起，收到第一帧有效数据后才解挂起开始工作。 */
#define RC_TICK_INIT_OFFSET   (RC_LOST_TIMEOUT_MS + 1U)

RC_Ctrl_t rc_ctrl;
volatile RC_Status_t rc_status;

/* 接收缓冲取 2 帧长度。IDLE 事件在总线空闲时触发，缓冲大于一帧才能稳定
   按“空闲”而不是“收满”结束，避免与下一帧粘连错位。 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t rc_rx_buf[RC_RX_BUF_LENGTH];
static uint8_t manual_rearm_hold_seen;

/* 整帧快照，避免 UART 中断更新到一半时混用通道与拨杆。 */
static RC_Ctrl_t Remote_Snapshot(void)
{
    RC_Ctrl_t snapshot;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    snapshot = rc_ctrl;
    __set_PRIMASK(primask);
    return snapshot;
}

static HAL_StatusTypeDef Remote_StartReceive(void)
{
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rc_rx_buf, RC_RX_BUF_LENGTH);

    ++rc_status.start_count;
    rc_status.last_start_status = (uint32_t)status;
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }
    return status;
}

/* ============================ 接收看门狗 ============================
 * 为什么需要它：STM32 HAL 里，DMA 模式下**任何**接收错误（帧错误 / 噪声 / 溢出 / 校验）
 * 都被当成 blocking error → UART_EndRxTransfer + HAL_DMA_Abort_IT（见 stm32h7xx_hal_uart.c
 * 的 HAL_UART_IRQHandler）。而 DMA1/DMA2 的 HAL_DMA_Abort_IT 只在 DMA 的**传输完成中断**
 * 到来时才回调（stm32h7xx_hal_dma.c 的 HAL_DMA_IRQHandler），NORMAL 模式 + IDLE 分帧时
 * DMA 从来走不满整缓冲，这个中断永远等不到 → UART_DMAAbortOnError / HAL_UART_ErrorCallback
 * 永不执行，写在 ErrorCallback 里的重新武装也就跑不到；而此刻 DMAR 已被清零、DMA EN=0，
 * 接收彻底死掉。
 *
 * 典型触发：冷启动时接收机与主板同时上电，接收机 TX 在自身启动期间电平未定义 → 连续帧错误
 * → 一次就把接收打死，表现为“遥控再也连不上、必须复位一次”。复位时接收机已在上电稳态、
 * 线路空闲高电平，不再出错，所以复位看起来“能修好”。
 *
 * 这里每轮检查“接收是否仍处于武装状态”，不是就强制重建：先轻量重建，失败再整段
 * DeInit/Init（因为 DMA 句柄会卡在 HAL_DMA_STATE_ABORT，HAL_DMA_Start_IT 会直接失败）。 */
static uint8_t Remote_IsArmed(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint8_t armed =
        (huart5.RxState == HAL_UART_STATE_BUSY_RX) &&
        ((huart5.Instance->CR3 & USART_CR3_DMAR) != 0U) &&
        (hdma_uart5_rx.State == HAL_DMA_STATE_BUSY);
    __set_PRIMASK(primask);
    return armed;
}

void Remote_Supervise(void)
{
    if (Remote_IsArmed() != 0U) {
        return;
    }

    ++rc_status.supervise_count;

    /* 轻量重建：清掉 UART 侧残留标志与状态，再重新武装 */
    HAL_UART_AbortReceive(&huart5);
    if (Remote_StartReceive() == HAL_OK) {
        return;
    }

    /* 轻量重建失败：DMA 句柄多半卡在 ABORT 态。整段 DeInit/Init 把 UART 与 DMA 都拉回
       干净状态（MspDeInit/MspInit 会重建 DMA 句柄并重新使能 NVIC 与时钟）。 */
    ++rc_status.reinit_count;
    (void)HAL_UART_DeInit(&huart5);
    MX_UART5_Init();
    (void)Remote_StartReceive();
}

/* 解析一整帧（18 字节）。拨杆值非法则判为坏帧，返回 0 不写入。 */
static uint8_t Remote_DecodeFrame(const uint8_t *f)
{
    uint8_t s1 = (uint8_t)((f[5] >> 4) & 0x03U);
    uint8_t s2 = (uint8_t)((f[5] >> 6) & 0x03U);

    /* DR16 上电初期/干扰会产生 s=0 的坏帧，直接丢弃 */
    if ((s1 == 0U) || (s2 == 0U)) {
        ++rc_status.invalid_frame_count;
        return 0U;
    }

    rc_ctrl.rc.ch0 = (int16_t)((f[0] | (f[1] << 8)) & 0x07FF) - RC_CH_VALUE_MID;
    rc_ctrl.rc.ch1 = (int16_t)(((f[1] >> 3) | (f[2] << 5)) & 0x07FF) - RC_CH_VALUE_MID;
    rc_ctrl.rc.ch2 = (int16_t)(((f[2] >> 6) | (f[3] << 2) | (f[4] << 10)) & 0x07FF) - RC_CH_VALUE_MID;
    rc_ctrl.rc.ch3 = (int16_t)(((f[4] >> 1) | (f[5] << 7)) & 0x07FF) - RC_CH_VALUE_MID;

    rc_ctrl.rc.s1 = s1;
    rc_ctrl.rc.s2 = s2;

    rc_ctrl.mouse.x = (int16_t)(f[6]  | (f[7]  << 8));
    rc_ctrl.mouse.y = (int16_t)(f[8]  | (f[9]  << 8));
    rc_ctrl.mouse.z = (int16_t)(f[10] | (f[11] << 8));
    rc_ctrl.mouse.press_l = f[12];
    rc_ctrl.mouse.press_r = f[13];

    rc_ctrl.key.v = (uint16_t)(f[14] | (f[15] << 8));

    rc_ctrl.rc.ch4 = (int16_t)((f[16] | (f[17] << 8)) & 0x07FF) - RC_CH_VALUE_MID;

    rc_ctrl.update_tick = HAL_GetTick();
    ++rc_status.valid_frame_count;
    return 1U;
}

void Remote_Init(void)
{
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));
    memset(rc_rx_buf, 0, sizeof(rc_rx_buf));
    rc_ctrl.update_tick = HAL_GetTick() - RC_TICK_INIT_OFFSET;
    manual_rearm_hold_seen = 0U;

    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    (void)Remote_StartReceive();
}

uint8_t Remote_IsOnline(void)
{
    return ((HAL_GetTick() - rc_ctrl.update_tick) <= RC_ONLINE_TIMEOUT_MS) ? 1U : 0U;
}

/* 连续 3s 没有有效帧就是遥控丢失（上电尚未收到第一帧时同样返回 1，所以系统默认挂起） */
uint8_t Remote_IsLost(void)
{
    return ((HAL_GetTick() - rc_ctrl.update_tick) > RC_LOST_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t Remote_IsHoldRequested(void)
{
    const RC_Ctrl_t snapshot = Remote_Snapshot();
    if ((HAL_GetTick() - snapshot.update_tick) > RC_LOST_TIMEOUT_MS)
    {
        return 1U;
    }
    /* 急停：两个拨杆都打下 */
    if ((snapshot.rc.s1 == RC_SW_DOWN) && (snapshot.rc.s2 == RC_SW_DOWN))
    {
        return 1U;
    }
    return 0U;
}

/* S2 上拨 = 上位机位置控制模式。Host 模块据此决定是否采纳下行位置目标；
   本函数只读拨杆状态，不参与失能/挂起判据。 */
uint8_t Remote_IsHostControl(void)
{
    const RC_Ctrl_t snapshot = Remote_Snapshot();
    return (((HAL_GetTick() - snapshot.update_tick) <= RC_ONLINE_TIMEOUT_MS) &&
            (snapshot.rc.s2 == RC_SW_UP)) ? 1U : 0U;
}

uint8_t Remote_ConsumeManualRearmRequest(void)
{
    const RC_Ctrl_t snapshot = Remote_Snapshot();
    const uint8_t online =
        ((HAL_GetTick() - snapshot.update_tick) <= RC_ONLINE_TIMEOUT_MS) ? 1U : 0U;
    const uint8_t sticks_neutral =
        (Remote_NormalizeChannel(snapshot.rc.ch0) == 0.0f &&
         Remote_NormalizeChannel(snapshot.rc.ch1) == 0.0f &&
         Remote_NormalizeChannel(snapshot.rc.ch2) == 0.0f &&
         Remote_NormalizeChannel(snapshot.rc.ch3) == 0.0f) ? 1U : 0U;

    if (online == 0U)
    {
        manual_rearm_hold_seen = 0U;
        return 0U;
    }
    if ((sticks_neutral == 0U) ||
        ((snapshot.rc.s1 == RC_SW_DOWN) && (snapshot.rc.s2 == RC_SW_DOWN)))
    {
        if ((sticks_neutral != 0U) &&
            (snapshot.rc.s1 == RC_SW_DOWN) && (snapshot.rc.s2 == RC_SW_DOWN))
        {
            manual_rearm_hold_seen = 1U;
        }
        return 0U;
    }
    if ((manual_rearm_hold_seen != 0U) && (snapshot.rc.s1 == RC_SW_MID) &&
        ((snapshot.rc.s2 == RC_SW_DOWN) || (snapshot.rc.s2 == RC_SW_UP)))
    {
        manual_rearm_hold_seen = 0U;
        return 1U;
    }
    return 0U;
}

float Remote_NormalizeChannel(int16_t value)
{
    if (value > (int16_t)RC_CHANNEL_FULL_SCALE) {
        value = (int16_t)RC_CHANNEL_FULL_SCALE;
    } else if (value < -(int16_t)RC_CHANNEL_FULL_SCALE) {
        value = -(int16_t)RC_CHANNEL_FULL_SCALE;
    }

    const float ratio = (float)value / RC_CHANNEL_FULL_SCALE;
    if (ratio > RC_CHANNEL_DEADBAND) return (ratio - RC_CHANNEL_DEADBAND) / (1.0f - RC_CHANNEL_DEADBAND);
    if (ratio < -RC_CHANNEL_DEADBAND) return (ratio + RC_CHANNEL_DEADBAND) / (1.0f - RC_CHANNEL_DEADBAND);
    return 0.0f;
}

/* 接收出错时 HAL 会结束接收，这里清错误标志并重新武装，否则接收会永久停住 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART5) {
        return;
    }

    ++rc_status.error_count;
    rc_status.last_error = huart->ErrorCode;
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* 先彻底停掉再重启，避免状态机停在 BUSY 导致重启返回 HAL_BUSY */
    HAL_UART_AbortReceive(huart);
    (void)Remote_StartReceive();
}

/* IDLE 或 DMA 传输完成时由 HAL 调用；Size 为本次实际收到的字节数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != UART5) {
        return;
    }

    ++rc_status.rx_event_count;
    rc_status.last_rx_size = Size;

    /* 从收到的数据里，取最后一个完整的 18 字节帧解析。
       正常每次空闲收到 1 帧 = 18 字节；若粘连收到多帧，取最新那帧。 */
    if (Size >= RC_FRAME_LENGTH) {
        uint16_t offset = Size - (Size % RC_FRAME_LENGTH) - RC_FRAME_LENGTH;
        (void)Remote_DecodeFrame(&rc_rx_buf[offset]);
    }

    /* 重新武装接收，等待下一帧 */
    (void)Remote_StartReceive();
}

/* 控制周期调用，归一化速度按各轴 remote_vel_max 换算 rad/s。
 * 通道分组（仅 S2 下拨时映射）：
 *   S1 中位 → ch0/ch1/ch3/ch2 = j1/j2/j3/j4；
 *   S1 上位 → ch0/ch1 = j5/j6，空闲的 ch3 = 夹爪 j7。
 * j1..j6 每拍提交全部六轴，包括零：回中、S2 中位/上拨、切组、100ms 离线撤销旧速度。
 * j7 是夹爪、不是速度轴：ch3 的正负向只用来切换两个预设位
 * （**正向 → 抓紧，负向 → 张开**，中位/死区内保持上一个状态），不经过 Joint。
 * S2 上拨 = 上位机位置模式：本函数不再映射摇杆，提交零后已接管的轴减速并保持，
 * 控制权交给上位机（位置目标走 Joint::SetStreamTarget，夹爪走 Gripper_Command）。
 * s1=s2=下仍由 Error_tasks 失能；这里立即撤销遥控运动指令。 */
void Remote_UpdateJointVelocity(void)
{
    float ratio[6] = {0.0f};
    const RC_Ctrl_t snapshot = Remote_Snapshot();

    /* 只有 S2 下拨才把摇杆映射成速度：S2 中位 = 遥控减速后保持，
       S2 上拨 = 上位机位置模式，两者都提交零速度。 */
    if (((HAL_GetTick() - snapshot.update_tick) <= RC_ONLINE_TIMEOUT_MS) &&
        (snapshot.rc.s2 == RC_SW_DOWN)) {
        if (snapshot.rc.s1 == RC_SW_MID) {
            ratio[0] = Remote_NormalizeChannel(snapshot.rc.ch0);
            ratio[1] = Remote_NormalizeChannel(snapshot.rc.ch1);
            ratio[2] = Remote_NormalizeChannel(snapshot.rc.ch3);
            ratio[3] = Remote_NormalizeChannel(snapshot.rc.ch2);
        } else if (snapshot.rc.s1 == RC_SW_UP) {
            const float gripper_ratio = Remote_NormalizeChannel(snapshot.rc.ch3);

            ratio[4] = Remote_NormalizeChannel(snapshot.rc.ch0);
            ratio[5] = Remote_NormalizeChannel(snapshot.rc.ch1);

            /* 夹爪用本组剩下的空闲通道（要换通道只改上一行）。
               用死区而不是 >0：杆停在中位附近时不会来回翻。 */
            if (gripper_ratio > RC_GRIPPER_DEADZONE)       Gripper_Command(1U);
            else if (gripper_ratio < -RC_GRIPPER_DEADZONE) Gripper_Command(0U);
        }
    }

    /* S2 冻结、未选中轴或失联产生的零指令不能冒充恢复后的真实回中。 */
    const uint8_t sticks_neutral = ((HAL_GetTick() - snapshot.update_tick) <= RC_ONLINE_TIMEOUT_MS) &&
        Remote_NormalizeChannel(snapshot.rc.ch0) == 0.0f && Remote_NormalizeChannel(snapshot.rc.ch1) == 0.0f &&
        Remote_NormalizeChannel(snapshot.rc.ch2) == 0.0f && Remote_NormalizeChannel(snapshot.rc.ch3) == 0.0f;
    for (unsigned char i = 0U; i < 6U; ++i) {
        Joint_SetRemoteVelocity(i, ratio[i], sticks_neutral);
    }
}
