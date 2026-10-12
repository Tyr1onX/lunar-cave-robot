#ifndef __HM_MOTOR_H
#define __HM_MOTOR_H

/**
 ******************************************************************************
 * @file    hm_motor.h
 * @brief   HM 系列关节伺服电机（G36）MIT 协议驱动
 *
 *          与 DM 电机协议核对结论（已逐项和 HM 手册 / 实机 Python 程序核对）：
 *            - MIT 控制帧布局相同：位置 16 bit，速度/Kp/Kd/力矩各 12 bit；
 *            - 特殊帧相同：使能 0xFC、待机 0xFD、设零 0xFE；
 *            - 量程不同：G36 为 P ±12.5 rad、V ±8 rad/s、T ±28 N·m、
 *              Kp 0~500、Kd 0~5（见 vehicle_config.h）；
 *            - 反馈帧：CAN ID 固定为主机 ID（0），数据字节 0 是整个电机 ID
 *              （不是 DM 的低 4 位 ID + 高 4 位状态）；
 *            - 反馈字节 6 = 电压余量（0~60 V 线性映射 8 bit），
 *              字节 7 = 温度（-20~150 ℃ 线性映射 8 bit），
 *              与 DM 的原始 MOS/线圈温度字节含义不同。
 ******************************************************************************
 */

#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"
#include "vehicle_config.h"

typedef struct
{
    unsigned short id;          /* 电机 CAN ID（数据字节 0） */
    float          pos;         /* 反馈位置 rad */
    float          vel;         /* 反馈速度 rad/s */
    float          tor;         /* 反馈力矩 N·m */
    float          volt;        /* 电压余量 V */
    float          temp;        /* 温度 ℃ */
    unsigned long  last_rx_ms;  /* 最近一次反馈的系统时刻 ms */
} HM_Motor_t;

/* MIT 协议无符号数映射（与 HM 手册一致） */
int   hm_float_to_uint(float x, float x_min, float x_max, int bits);
float hm_uint_to_float(int x_int, float x_min, float x_max, int bits);

/* MIT 控制帧编码：pos 16 bit，vel/kp/kd/tor 各 12 bit，输出 8 字节 */
void hm_motor_encode_mit(uint8_t data[8],
                         float pos, float vel, float kp, float kd, float tor);

/* 发送完整 MIT 指令 */
void hm_motor_send_mit(hcan_t *hcan, unsigned short motor_id,
                       float pos, float vel, float kp, float kd, float tor);

/* MIT 速度模式：P_des=0、Kp=0、T_ff=0，Kd 取 G36_VELOCITY_KD */
void hm_motor_send_velocity(hcan_t *hcan, unsigned short motor_id, float vel);

/* 全零指令：Kp=Kd=0、V=0、T=0，电机处于放松状态但仍回传反馈 */
void hm_motor_send_relax(hcan_t *hcan, unsigned short motor_id);

/* 特殊帧：使能 / 待机 / 当前位置设零 */
void hm_motor_enable(hcan_t *hcan, unsigned short motor_id);
void hm_motor_disable(hcan_t *hcan, unsigned short motor_id);
void hm_motor_set_zero(hcan_t *hcan, unsigned short motor_id);

/* 反馈帧解码。data 为 8 字节，返回解码出的电机 ID；帧非法时返回 0xFF。 */
unsigned short hm_motor_decode_feedback(HM_Motor_t *motor,
                                        const uint8_t *data, unsigned long data_len,
                                        unsigned long now_ms);

#endif /* __HM_MOTOR_H */
