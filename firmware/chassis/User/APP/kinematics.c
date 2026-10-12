/**
 ******************************************************************************
 * @file    kinematics.c
 * @brief   6 轮差速车运动学解算（详见 kinematics.h 的符号约定）
 ******************************************************************************
 */

#include "kinematics.h"

static float clampf(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

float kinematics_side_to_motor(float side_speed_ms, float dir)
{
    /* 由 v=omega*r 得 omega=v/r；dir 只处理左右电机的镜像安装。 */
    return side_speed_ms / WHEEL_RADIUS_M * dir;
}

float kinematics_motor_to_side(float motor_speed_radps, float dir)
{
    return motor_speed_radps * WHEEL_RADIUS_M * dir;
}

void kinematics_inverse(float v_x_ms, float turn_out_ms,
                        float *v_left_ms, float *v_right_ms)
{
    /* turn_out 是左右速度差的一半，因此 v_left-v_right=2*turn_out。 */
    *v_left_ms  = v_x_ms + turn_out_ms;
    *v_right_ms = v_x_ms - turn_out_ms;
}

void kinematics_forward(float v_left_ms, float v_right_ms,
                        float *v_x_ms, float *omega_radps)
{
    /*
     * 线速度取两侧平均；速度差除以轮距得到理想偏航角速度。
     * 六轮滑移转向会侧滑，所以闭环航向仍以 IMU 为准。
     */
    *v_x_ms     = (v_left_ms + v_right_ms) * 0.5f;
    *omega_radps = (v_right_ms - v_left_ms) / VEHICLE_TRACK_WIDTH_M;
}

void kinematics_setpoints_6wd(float v_x_ms, float turn_out_ms,
                              float motor_speed_radps[WHEEL_COUNT])
{
    float v_left;
    float v_right;
    float motor_limit;
    unsigned char i;

    kinematics_inverse(v_x_ms, turn_out_ms, &v_left, &v_right);

    /* 单侧线速度限幅，防止转向叠加后超过电机能力 */
    v_left  = clampf(v_left, SIDE_SPEED_MAX_MS);
    v_right = clampf(v_right, SIDE_SPEED_MAX_MS);

    /* 协议层再留 5% 余量，避免浮点换算后落到 MIT 量程边界。 */
    motor_limit = G36_V_MAX * 0.95f;

    for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
    {
        float side = (i <= (unsigned char)WHEEL_LEFT_REAR) ? v_left : v_right;
        float speed = kinematics_side_to_motor(side, g_wheel_motor_config[i].dir);
        motor_speed_radps[i] = clampf(speed, motor_limit);
    }
}

void kinematics_feedback_6wd(const float motor_vel_radps[WHEEL_COUNT],
                             float *v_left_ms, float *v_right_ms,
                             float *v_x_ms, float *omega_radps)
{
    float left_sum = 0.0f;
    float right_sum = 0.0f;
    float v_left;
    float v_right;
    unsigned char i;

    for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
    {
        float side = kinematics_motor_to_side(motor_vel_radps[i],
                                              g_wheel_motor_config[i].dir);
        if (i <= (unsigned char)WHEEL_LEFT_REAR)
        {
            left_sum += side;
        }
        else
        {
            right_sum += side;
        }
    }

    /* 同侧三轮理论同速，使用平均值可减弱单轮打滑和反馈噪声。 */
    v_left  = left_sum / 3.0f;
    v_right = right_sum / 3.0f;

    if (v_left_ms != 0)
    {
        *v_left_ms = v_left;
    }
    if (v_right_ms != 0)
    {
        *v_right_ms = v_right;
    }
    if (v_x_ms != 0 || omega_radps != 0)
    {
        float v_x;
        float omega;
        kinematics_forward(v_left, v_right, &v_x, &omega);
        if (v_x_ms != 0)
        {
            *v_x_ms = v_x;
        }
        if (omega_radps != 0)
        {
            *omega_radps = omega;
        }
    }
}
