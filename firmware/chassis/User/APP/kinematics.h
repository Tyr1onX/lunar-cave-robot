#ifndef __KINEMATICS_H
#define __KINEMATICS_H

/**
 ******************************************************************************
 * @file    kinematics.h
 * @brief   6 轮差速车运动学解算
 *
 *          车体模型：左右两侧各 3 个车轮，同侧 3 轮速度始终相同（滑移转向）。
 *          正运动学：由左右侧线速度求车体线速度 v_x 和偏航角速度 omega。
 *          逆运动学：由 v_x 和转向修正量 turn_out 求 6 个电机的角速度指令。
 *
 *          符号约定（俯视）：
 *            - 车体前进方向为 x 轴正方向，v_x > 0 为前进；
 *            - 偏航角 yaw 逆时针为正，omega > 0 为左转；
 *            - v_left / v_right 为左/右侧车轮的接地线速度（m/s）；
 *            - omega = (v_right - v_left) / VEHICLE_TRACK_WIDTH_M。
 ******************************************************************************
 */

#include "vehicle_config.h"

/* 单侧线速度 (m/s) -> 电机输出轴角速度 (rad/s)，含安装方向 */
float kinematics_side_to_motor(float side_speed_ms, float dir);

/* 电机输出轴角速度 (rad/s) -> 单侧线速度 (m/s)，含安装方向 */
float kinematics_motor_to_side(float motor_speed_radps, float dir);

/* 逆运动学：v_x + 转向修正 -> 左右侧线速度
 *   v_left  = v_x + turn_out
 *   v_right = v_x - turn_out
 * （turn_out > 0 时左快右慢，车体顺时针修正，yaw 减小） */
void kinematics_inverse(float v_x_ms, float turn_out_ms,
                        float *v_left_ms, float *v_right_ms);

/* 正运动学：左右侧线速度 -> 车体 v_x 与偏航角速度 omega */
void kinematics_forward(float v_left_ms, float v_right_ms,
                        float *v_x_ms, float *omega_radps);

/* 逆解并展开到 6 个电机：
 * 输入  v_x_ms     车体系线速度指令 (m/s)
 *       turn_out_ms 转向控制器输出的差速修正量 (m/s)
 * 输出  motor_speed_radps[WHEEL_COUNT]  按 wheel_index_e 顺序的电机角速度 (rad/s)
 * 内部完成：左右侧分配、单侧限幅、轮径换算、逐电机方向符号、电机量程限幅。 */
void kinematics_setpoints_6wd(float v_x_ms, float turn_out_ms,
                              float motor_speed_radps[WHEEL_COUNT]);

/* 由 6 个电机的反馈角速度解算车体状态：
 * 输入  motor_vel_radps[WHEEL_COUNT]  按 wheel_index_e 顺序的电机反馈角速度 (rad/s)
 * 输出  v_left_ms / v_right_ms / v_x_ms / omega_radps（不需要的项可传 0） */
void kinematics_feedback_6wd(const float motor_vel_radps[WHEEL_COUNT],
                             float *v_left_ms, float *v_right_ms,
                             float *v_x_ms, float *omega_radps);

#endif /* __KINEMATICS_H */
