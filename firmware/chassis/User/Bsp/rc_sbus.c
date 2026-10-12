/**
 ******************************************************************************
 * @file    rc_sbus.c
 * @brief   HOT-10RC10A SBUS 遥控接收实现
 *
 *          使用 UART5（PD2）以 100kbps 8E2 接收 SBUS 帧，
 *          DMA 空闲中断方式接收，在 HAL_UARTEx_RxEventCallback 中解析。
 *
 *          SBUS 帧格式（25 bytes）：
 *            [0]   = 0x0F     帧头
 *            [1..22] = 16 通道数据，每通道 11 bit
 *            [23]  = 标志字节（bit6=丢失帧，bit7=失效）
 *            [24]  = 0x00     帧尾
 ******************************************************************************
 */

#include "rc_sbus.h"
#include "rc_control_mode.h"
#include "usart.h"
#include "string.h"
#include "Chaiss_Communication.h"

rc_sbus_t rc_sbus;

/* 通道映射表（Mode 2：左杆上下=油门，右杆左右=转向） */
const sbus_ch_map_t g_sbus_ch_map = {
    .throttle = 2,  /* CH3 = 左摇杆上下 */
    .steer    = 0,  /* CH1 = 右摇杆左右 */
    .heading_lock = 6, /* CH7 = SWC（如实测通道不同，只改这里） */
    .control_mode = 7, /* CH8 = 下拨失能/中置串口/上拨遥控 */
};

/* SBUS DMA 接收缓冲区 */
static uint8_t sbus_rx_buff[SBUS_DMA_BUFFER_SIZE];

/* SBUS 原始值 -> PWM 脉宽转换（1000~2000 us） */
static inline uint16_t sbus_to_pwm(uint16_t sbus_val)
{
    return (uint16_t)((sbus_val * 1000U) / 2047U + 1000U);
}

/* SBUS 帧解析（与备份工程 uart_bsp.c 一致） */
static void sbus_frame_parse(rc_sbus_t *sbus, uint8_t *buf)
{
    uint16_t ch[10];
    int i;

    if ((buf[0] != 0x0F) || (buf[24] != 0x00))
        return;

    /* 标志字节：bit6=1 表示丢失帧 */
    if ((buf[23] & 0x0Cu) != 0u)
        sbus->online = 0;
    else
        sbus->online = 1;

    /* 解出 10 个通道（SBUS 标准 11-bit 位打包） */
    ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
    ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
    ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
    ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);

    for (i = 0; i < 10; i++)
    {
        sbus->ch[i] = ch[i];
        sbus->ch_pwm[i] = sbus_to_pwm(ch[i]);
    }
    sbus->last_frame_ms = HAL_GetTick();
}

void rc_sbus_init(void)
{
    /* 启动 UART5 DMA 空闲中断接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buff, SBUS_DMA_BUFFER_SIZE);
}

void rc_sbus_update(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - rc_sbus.last_frame_ms) > SBUS_FRAME_TIMEOUT_MS)
    {
        rc_sbus.online = 0;
    }
}

float rc_sbus_channel_norm(uint8_t ch)
{
    int val;
    int delta;
    int half_span;

    if (ch >= 10 || rc_sbus.ch[ch] == 0)
    {
        return 0.0f;
    }

    val = (int)rc_sbus.ch[ch];
    delta = val - (int)SBUS_CH_MID;

    /* 死区处理 */
    if (delta > -(int)SBUS_CH_DEADBAND && delta < (int)SBUS_CH_DEADBAND)
    {
        return 0.0f;
    }

    half_span = (int)SBUS_CH_MAX - (int)SBUS_CH_MID;
    float norm = (float)delta / (float)half_span;

    if (norm > 1.0f)
        norm = 1.0f;
    else if (norm < -1.0f)
        norm = -1.0f;

    return norm;
}

/* UART5 DMA 空闲中断回调：SBUS 帧接收完成 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        /* 重启 DMA 接收 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buff, SBUS_DMA_BUFFER_SIZE);

        if (Size <= SBUS_BUFF_SIZE)
        {
            sbus_frame_parse(&rc_sbus, sbus_rx_buff);
        }
        else
        {
            memset(sbus_rx_buff, 0, sizeof(sbus_rx_buff));
        }
    }
    else if (huart->Instance == USART1)
    {
        Chaiss_Communication_RxEvent(Size);
    }
}

/* UART5 错误回调：DMA 接收恢复 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buff, SBUS_DMA_BUFFER_SIZE);
        memset(sbus_rx_buff, 0, sizeof(sbus_rx_buff));
    }
    else if (huart->Instance == USART1)
    {
        Chaiss_Communication_UARTError();
    }
}


