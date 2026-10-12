/**
 ******************************************************************************
 * @file    rc_sbus.h
 * @brief   HOT-10RC10A SBUS 遥控接收（UART5, PD2, 100kbps 8E2）
 *
 *          SBUS 帧：25 字节，帧头 0x0F，帧尾 0x00，16 通道各 11 bit。
 *          接收机：HOT-10RC10A，SBUS 协议，接在 PD2 = UART5_RX。
 *
 *          摇杆映射（默认为 Mode 2）：
 *            CH1 (右摇杆左右) -> 转向
 *            CH3 (左摇杆上下) -> 前进/后退
 *            CH7 (开关/旋钮)  -> 航向锁定
 *            CH8 (三段拨杆)    -> 失能/串口/遥控控制源选择
 *
 *          参考：DM-car-lqr-position_backup_20260723 的 uart_bsp.c
 ******************************************************************************
 */

#ifndef __RC_SBUS_H
#define __RC_SBUS_H

#include "main.h"
#include "rc_control_mode.h"

#define SBUS_BUFF_SIZE          25      /* SBUS 帧长度 */
#define SBUS_DMA_BUFFER_SIZE    (SBUS_BUFF_SIZE * 2u)
#define SBUS_FRAME_TIMEOUT_MS   100     /* 帧超时判掉线 ms */

/* SBUS 通道原始值范围（11 位 ADC 值） */
#define SBUS_CH_MIN             172
#define SBUS_CH_MAX             1811
#define SBUS_CH_MID             991
#define SBUS_CH_DEADBAND        20

/* 航向锁定阈值（对应 SBUS 通道 PWM 值） */
#define SBUS_HEADING_LOCK_THRESHOLD 1500

/* 逻辑通道映射表：可在此处修改通道映射 */
typedef struct
{
    uint8_t throttle;   /* 油门通道索引（左摇杆上下） */
    uint8_t steer;      /* 转向通道索引（右摇杆左右） */
    uint8_t control_mode; /* 三段拨杆通道索引（CH8） */
    uint8_t heading_lock; /* 航向锁定通道索引（CH7） */
} sbus_ch_map_t;

/* SBUS 接收数据结构 */
typedef struct
{
    uint16_t online;            /* 1 = 在线，0 = 掉线 */
    uint16_t ch[10];            /* 原始通道值（0~2047） */
    uint16_t ch_pwm[10];        /* 转换为 PWM 脉宽值（1000~2000 us） */
    uint32_t last_frame_ms;     /* 最近一帧接收时间 ms */
} rc_sbus_t;

extern rc_sbus_t rc_sbus;
extern const sbus_ch_map_t g_sbus_ch_map;

/* 初始化 SBUS（启动 UART5 DMA 接收）*/
void rc_sbus_init(void);

/* 周期调用（建议 10ms）：更新 online 掉线标志 */
void rc_sbus_update(void);

/* 通道归一化到 -1.0 ~ +1.0，含中位死区和限幅 */
float rc_sbus_channel_norm(uint8_t ch);

#endif /* __RC_SBUS_H */
