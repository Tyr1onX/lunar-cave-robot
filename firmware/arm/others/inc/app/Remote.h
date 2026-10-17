#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_FRAME_LENGTH   18U
#define RC_RX_BUF_LENGTH  36U

#define RC_SW_UP          1U
#define RC_SW_MID         3U
#define RC_SW_DOWN        2U

#define RC_CH_VALUE_MID   1024

/* 连续无有效遥控数据超过该时间 -> 判定遥控丢失，系统挂起（上电未接遥控也算） */
#define RC_LOST_TIMEOUT_MS   3000U

/* 夹爪通道的切换死区（归一化摇杆量）：|ratio| 超过它才切换张开/抓紧，
   中位附近抖动不会来回翻转；回中保持上一个状态。 */
#define RC_GRIPPER_DEADZONE  0.1f

typedef struct
{
    struct
    {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        int16_t ch4;
        uint8_t s1;
        uint8_t s2;
    } rc;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct
    {
        uint16_t v;
    } key;

    uint32_t update_tick;
} RC_Ctrl_t;

typedef struct
{
    uint32_t start_count;
    uint32_t rx_event_count;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t error_count;
    uint32_t supervise_count;   /* 接收看门狗发现「未武装」并尝试重建的次数 */
    uint32_t reinit_count;      /* 轻量重建失败、整段 DeInit/Init 的次数 */
    uint32_t last_error;
    uint32_t last_start_status;
    uint16_t last_rx_size;
} RC_Status_t;

extern RC_Ctrl_t rc_ctrl;
extern volatile RC_Status_t rc_status;

void Remote_Init(void);

/* 接收看门狗：由 Error_tasks 周期调用。UART5 的 IDLE+DMA 接收一旦因错误被打死，
   HAL 的恢复链不保证会回调（见 Remote.c 顶部 Remote_Supervise 的说明），
   这里兜底强制重建，避免出现“必须复位才能接遥控”的现象。 */
void Remote_Supervise(void);

/* 遥控数据“在线”：100ms 内有有效帧（否则遥控速度命令归零） */
uint8_t Remote_IsOnline(void);

/* 遥控“丢失”：连续 RC_LOST_TIMEOUT_MS 无有效帧。上电后未收到第一帧时也返回 1 */
uint8_t Remote_IsLost(void);

/* 系统是否应挂起：遥控丢失 或 急停（s1/s2 都打下）。
   由 Error_tasks 周期轮询，决定挂起/解挂起整个执行任务与电机使能 */
uint8_t Remote_IsHoldRequested(void);

/* S2 上拨 = 上位机位置控制模式：遥控侧不再映射摇杆（提交零速度让出已接管的轴），
   位置目标由上位机经 Joint::SetTarget() 下发。Host 模块据此决定是否采纳下行目标。 */
uint8_t Remote_IsHostControl(void);

/* Deliberate re-arm gesture: with neutral sticks, move S1 and S2 to DOWN/DOWN,
   then move S1 to MID and select S2 DOWN (manual RC) or UP (host control). */
uint8_t Remote_ConsumeManualRearmRequest(void);

float Remote_NormalizeChannel(int16_t value);

/* roboARM_task 每控制周期调用：整帧快照映射六轴（j1..j6）归一化速度。
   S2 下拨才做摇杆映射；S2 中位（遥控减速后保持）、上拨（上位机位置模式）、
   未选中轴、回中和离线都显式提交零。
   j7 是夹爪，不是速度轴：在同一组里把 ch3 的正负向直接转成夹爪命令（见下）。 */
void Remote_UpdateJointVelocity(void);

/* 仅控制任务调用：ratio × 各轴 remote_vel_max 得到 rad/s，输入层不积分。
   idx: 0..5 -> j1..j6（夹爪不走本入口）。 */
void Joint_SetRemoteVelocity(unsigned char idx, float ratio, unsigned char sticks_neutral);

/* 夹爪（j7）：1 = 抓紧，0 = 张开。遥控 ch3 正向发 1、负向发 0；
   上位机的夹爪命令字节也调这里（见 Host.cpp）。两处互斥（S2 上/下拨不会同时生效）。 */
void Gripper_Command(unsigned char close_target);


#ifdef __cplusplus
}
#endif

#endif
