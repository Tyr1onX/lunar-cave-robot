#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

/**
 ******************************************************************************
 * @file    chassis_task.h
 * @brief   6 轮差速移动车底盘任务
 *
 *          整车结构：左右两侧各 3 个车轮，同侧 3 轮速度相同，差速转向。
 *          电机：HM G36，MIT 协议速度模式；左侧 3 个电机接 CAN1，
 *          右侧 3 个电机接 CAN2（见 vehicle_config.h 的电机配置表）。
 *
 *          转向控制器：
 *            error = angular_z_cmd - yaw_rate_imu
 *            omega = angular_z_cmd + KP*error + KI*integral(error)
 *          omega 经轮距换算为左右轮差速，前馈负责基本转向，IMU 反馈
 *          补偿轮胎打滑和左右负载差。
 *
 *          与外部的接口：
 *            - 遥控或未来机械臂通过 chassis_command_publish() 发布 x/z；
 *            - CAN 接收中断（can_bsp.c）写入 motor[] 反馈；
 *            - 本任务读取 INS 的 YawTotalAngle / Gyro[2] 做闭环。
 ******************************************************************************
 */

#include "main.h"
#include "hm_motor.h"
#include "vehicle_config.h"
#include "INS_task.h"
#include "chassis_command.h"

typedef struct
{
    HM_Motor_t motor[WHEEL_COUNT]; /* 6 个车轮电机反馈，按 wheel_index_e 顺序 */

    float v_set;         /* 车体系目标线速度 m/s */
    float yaw_rate_set;  /* 目标偏航角速度 rad/s */

    float v;        /* 轮速解算的车体线速度 m/s */
    float v_left;   /* 左侧平均接地线速度 m/s */
    float v_right;  /* 右侧平均接地线速度 m/s */
    float omega;    /* 轮速解算的偏航角速度 rad/s */

    float yaw;      /* 当前总偏航角 rad，来自 INS */
    float d_yaw;    /* 偏航角速度 rad/s，来自陀螺仪 Z 轴 */
    float turn_out; /* 转向控制器输出 m/s */
    float yaw_rate_error;
    float yaw_rate_control;
    float heading_lock_target;
    float heading_lock_error;
    uint8_t heading_lock_active;
    chassis_cmd_source_e command_source;

    uint8_t start_flag; /* 1=有效且未超时的速度指令，0=放松 */
} chassis_t;

extern chassis_t chassis_move;

void Chassis_init(chassis_t *chassis);
void Chassis_task(void);

#endif /* __CHASSIS_TASK_H */
