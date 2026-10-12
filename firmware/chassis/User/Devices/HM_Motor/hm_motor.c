/**
 ******************************************************************************
 * @file    hm_motor.c
 * @brief   HM 系列关节伺服电机（G36）MIT 协议驱动实现
 ******************************************************************************
 */

#include "hm_motor.h"

#define HM_VB_MIN     (0.0f)    /* 电压余量映射范围 V */
#define HM_VB_MAX     (60.0f)
#define HM_TEMP_MIN   (-20.0f)  /* 温度映射范围 ℃ */
#define HM_TEMP_MAX   (150.0f)

#define HM_CMD_ENABLE   (0xFCu) /* 使能 */
#define HM_CMD_STANDBY  (0xFDu) /* 待机 */
#define HM_CMD_SET_ZERO (0xFEu) /* 当前位置设零 */

int hm_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;

    /* 超量程钳位，避免溢出后打包出错误报文 */
    if (x > x_max)
    {
        x = x_max;
    }
    else if (x < x_min)
    {
        x = x_min;
    }
    /* 线性映射到 [0, 2^bits-1]；例如速度 -8~+8 对应 12 bit 的 0~4095。 */
    return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

float hm_uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

void hm_motor_encode_mit(uint8_t data[8],
                         float pos, float vel, float kp, float kd, float tor)
{
    unsigned short pos_tmp;
    unsigned short vel_tmp;
    unsigned short kp_tmp;
    unsigned short kd_tmp;
    unsigned short tor_tmp;

    pos_tmp = (unsigned short)hm_float_to_uint(pos, G36_P_MIN, G36_P_MAX, 16);
    vel_tmp = (unsigned short)hm_float_to_uint(vel, G36_V_MIN, G36_V_MAX, 12);
    kp_tmp  = (unsigned short)hm_float_to_uint(kp, G36_KP_MIN, G36_KP_MAX, 12);
    kd_tmp  = (unsigned short)hm_float_to_uint(kd, G36_KD_MIN, G36_KD_MAX, 12);
    tor_tmp = (unsigned short)hm_float_to_uint(tor, G36_T_MIN, G36_T_MAX, 12);

    /*
     * MIT 控制帧：P_des 占 16 bit，V_des/Kp/Kd/T_ff 各占 12 bit。
     * 12 bit 字段会跨两个字节，下面严格按 HM 手册规定的高位在前方式打包。
     */
    data[0] = (uint8_t)(pos_tmp >> 8);
    data[1] = (uint8_t)(pos_tmp);
    data[2] = (uint8_t)(vel_tmp >> 4);
    data[3] = (uint8_t)(((vel_tmp & 0xFu) << 4) | (kp_tmp >> 8));
    data[4] = (uint8_t)(kp_tmp);
    data[5] = (uint8_t)(kd_tmp >> 4);
    data[6] = (uint8_t)(((kd_tmp & 0xFu) << 4) | (tor_tmp >> 8));
    data[7] = (uint8_t)(tor_tmp);
}

void hm_motor_send_mit(hcan_t *hcan, unsigned short motor_id,
                       float pos, float vel, float kp, float kd, float tor)
{
    uint8_t data[8];

    hm_motor_encode_mit(data, pos, vel, kp, kd, tor);
    canx_send_data(hcan, motor_id, data, 8u);
}

void hm_motor_send_velocity(hcan_t *hcan, unsigned short motor_id, float vel)
{
    /*
     * MIT 纯速度模式：Kp=0 关闭位置环；Kd 与 V_des 构成速度调节；
     * T_ff=0 表示不加前馈力矩。此 Kd 与整车 IMU 角速度闭环参数无关。
     */
    hm_motor_send_mit(hcan, motor_id, 0.0f, vel, 0.0f, G36_VELOCITY_KD, 0.0f);
}

void hm_motor_send_relax(hcan_t *hcan, unsigned short motor_id)
{
    hm_motor_send_mit(hcan, motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

static void hm_motor_send_special(hcan_t *hcan, unsigned short motor_id, uint8_t cmd)
{
    uint8_t data[8];
    unsigned char i;

    for (i = 0u; i < 7u; i++)
    {
        data[i] = 0xFFu;
    }
    data[7] = cmd;
    canx_send_data(hcan, motor_id, data, 8u);
}

void hm_motor_enable(hcan_t *hcan, unsigned short motor_id)
{
    hm_motor_send_special(hcan, motor_id, HM_CMD_ENABLE);
}

void hm_motor_disable(hcan_t *hcan, unsigned short motor_id)
{
    hm_motor_send_special(hcan, motor_id, HM_CMD_STANDBY);
}

void hm_motor_set_zero(hcan_t *hcan, unsigned short motor_id)
{
    hm_motor_send_special(hcan, motor_id, HM_CMD_SET_ZERO);
}

unsigned short hm_motor_decode_feedback(HM_Motor_t *motor,
                                        const uint8_t *data, unsigned long data_len,
                                        unsigned long now_ms)
{
    int p_int;
    int v_int;
    int t_int;

    if (data_len != 8u)
    {
        return 0xFFu;
    }

    /* HM 协议：数据字节 0 整个就是电机 ID（1~127），不含状态位 */
    motor->id = data[0];

    p_int = ((int)data[1] << 8) | (int)data[2];
    v_int = ((int)data[3] << 4) | ((int)data[4] >> 4);
    t_int = (((int)data[4] & 0xF) << 8) | (int)data[5];

    motor->pos  = hm_uint_to_float(p_int, G36_P_MIN, G36_P_MAX, 16);
    motor->vel  = hm_uint_to_float(v_int, G36_V_MIN, G36_V_MAX, 12);
    motor->tor  = hm_uint_to_float(t_int, G36_T_MIN, G36_T_MAX, 12);
    motor->volt = hm_uint_to_float((int)data[6], HM_VB_MIN, HM_VB_MAX, 8);
    motor->temp = hm_uint_to_float((int)data[7], HM_TEMP_MIN, HM_TEMP_MAX, 8);

    motor->last_rx_ms = now_ms;
    return motor->id;
}
