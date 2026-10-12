#ifndef __VEHICLE_CONFIG_H
#define __VEHICLE_CONFIG_H

/**
 ******************************************************************************
 * @file    vehicle_config.h
 * @brief   6 轮差速车整车参数 + HM G36 电机配置（唯一配置文件）
 *
 *          整车：6 轮移动车，每侧 3 个车轮，差速转向。
 *          电机：HM G36 关节伺服电机，MIT 协议速度模式。
 *          左侧 3 个电机接 CAN1，右侧 3 个电机接 CAN2。
 *
 *          需要现场标定的项已用 [标定] 标出。
 ******************************************************************************
 */

/* ================= 整车几何参数 ================= */

#define VEHICLE_TRACK_WIDTH_M   0.535f  /* 轮间距（左右车轮中心面距离），单位 m */
#define WHEEL_DIAMETER_M        0.248f  /* 车轮直径，单位 m */
#define WHEEL_RADIUS_M          (WHEEL_DIAMETER_M * 0.5f)  /* 车轮半径 0.124 m */

/* ================= G36 电机协议量程 =================
 * 与电机内部寄存器 P_MAX/V_MAX/T_MAX 一致（实机回传 12.5 / 8.0 / 28.0）。
 * MIT 编解码必须使用这组量程，否则数值会解错。 */

#define G36_P_MIN               (-12.5f)   /* 位置量程 rad */
#define G36_P_MAX               (12.5f)
#define G36_V_MIN               (-8.0f)    /* 速度量程 rad/s（输出轴） */
#define G36_V_MAX               (8.0f)
#define G36_KP_MIN              (0.0f)     /* Kp 量程 */
#define G36_KP_MAX              (500.0f)
#define G36_KD_MIN              (0.0f)     /* Kd 量程 */
#define G36_KD_MAX              (5.0f)
#define G36_T_MIN               (-28.0f)   /* 力矩量程 N·m */
#define G36_T_MAX               (28.0f)

/*
 * G36 内部速度环参数（MIT 报文中的 Kd）：
 * 速度模式使用 P_des=0、Kp=0、V_des=目标角速度、T_ff=0。此时 Kd
 * 决定电机纠正速度误差的力度，它不是整车航向 PD 中的 YAW_KD。
 *
 * 太小：电机发软、带载掉速；太大：启动冲击、电流和噪声增大，甚至抖动。
 * 建议架空车轮，从 0.5 起每次增加 0.2~0.3，再进行低速带载测试。
 * 当前 2.1 是厂家推荐起点，最终值需根据整车负载标定。
 */
#define G36_VELOCITY_KD         (60.2f)

/*
 * 出厂默认反馈 ID 为 MASTER_ID=0。本车已改用独立反馈 ID：
 * feedback_id = 0x10 + can_id，即 0x11~0x16。
 * 此常量仅保留作协议资料，当前反馈分发见 can_bsp.c。
 */
#define HM_MASTER_ID            (0u)

/* ================= 电机安装配置 =================
 * 左前 ID=1、左中 ID=3、左后 ID=5  -> CAN1
 * 右前 ID=2、右中 ID=4、右后 ID=6  -> CAN2
 * 左右电机关于中间平面对称安装，同一转向指令下两侧电机旋转方向相反，
 * 用 dir 逐电机配置符号：dir = +1 表示电机正转 = 车轮前进。[标定] */

typedef enum
{
    MOTOR_CAN1 = 0,
    MOTOR_CAN2 = 1
} motor_can_bus_e;

typedef enum
{
    WHEEL_LEFT_FRONT = 0,   /* 左前  CAN1 ID=1 */
    WHEEL_LEFT_MID,         /* 左中  CAN1 ID=3 */
    WHEEL_LEFT_REAR,        /* 左后  CAN1 ID=5 */
    WHEEL_RIGHT_FRONT,      /* 右前  CAN2 ID=2 */
    WHEEL_RIGHT_MID,        /* 右中  CAN2 ID=4 */
    WHEEL_RIGHT_REAR,       /* 右后  CAN2 ID=6 */
    WHEEL_COUNT
} wheel_index_e;

typedef struct
{
    motor_can_bus_e bus;    /* 所在 CAN 总线 */
    unsigned short  can_id; /* MIT 模式电机 CAN ID */
    float           dir;    /* 安装方向符号，+1.0f 或 -1.0f */
} wheel_motor_config_t;

/* 电机配置表：改 ID、总线或方向只需要改这里。
 * static const 放在头文件中，保证"电机配置只在这一个文件里"。 */
static const wheel_motor_config_t g_wheel_motor_config[WHEEL_COUNT] =
{
    /* bus        id   dir */
    { MOTOR_CAN1,  1u,  +1.0f },   /* WHEEL_LEFT_FRONT  */
    { MOTOR_CAN1,  3u,  +1.0f },   /* WHEEL_LEFT_MID    */
    { MOTOR_CAN1,  5u,  +1.0f },   /* WHEEL_LEFT_REAR   */
    { MOTOR_CAN2,  2u,  -1.0f },   /* WHEEL_RIGHT_FRONT */
    { MOTOR_CAN2,  4u,  -1.0f },   /* WHEEL_RIGHT_MID   */
    { MOTOR_CAN2,  6u,  -1.0f },   /* WHEEL_RIGHT_REAR  */
};

/* ================= 控制参数 ================= */

#define CHASSIS_TASK_PERIOD_MS  (5u)      /* 底盘控制周期 ms（200 Hz） */

/* ================= IMU yaw-rate closed loop =================
 *
 * error         = angular_z_cmd - yaw_rate_imu
 * omega_control = angular_z_cmd + KP*error + KI*integral(error)
 *
 * angular_z_cmd is kinematic feed-forward. The feedback term corrects slip
 * and unequal wheel loading. Start with KI=0 and tune KP at low speed.
 */
#define YAW_RATE_KP                    (0.30f)
#define YAW_RATE_KI                    (0.00f)
#define YAW_RATE_INTEGRAL_LIMIT        (0.50f)
#define YAW_RATE_CORRECTION_MAX_RADPS  (0.80f)
#define IMU_YAW_RATE_SIGN              (+1.0f) /* Set -1 if IMU Z sign is opposite. */
#define IMU_YAW_RATE_LPF_ALPHA         (0.15f)

/*
 * Heading-lock outer loop: yaw error -> yaw-rate request.
 * This maximum return rate is independent of normal driving's
 * YAW_RATE_MAX_RADPS. Change it to tune only disturbance recovery speed.
 */
#define HEADING_LOCK_KP                       (9.5f)
#define HEADING_LOCK_RETURN_RATE_MAX_RADPS    (2.0f)

#define V_CMD_MAX_MS            (0.6f)    /* 最大车体线速度 m/s */
#define V_CMD_RAMP_MPS2         (2.0f)    /* 线速度指令加速度限制 m/s^2 */
#define YAW_RATE_MAX_RADPS      (1.5f)    /* 车绕自身 Z 轴的最大偏航角速度 rad/s */
#define RC_STEER_SIGN           (-1.0f)   /* 遥控器右打应产生顺时针（负 angular_z） */
#define YAW_RATE_RAMP_RADPS2    (3.0f)    /* 普通运动目标角速度斜坡 rad/s^2 */
#define CHASSIS_CMD_TIMEOUT_MS  (200u)
#define ENABLE_CHASSIS_AUTO_TEST (0u)     /* Only set to 1 with wheels lifted. */
/* 左右任一侧车轮允许的最大接地线速度。 */
#define SIDE_SPEED_MAX_MS       (0.6f)

#endif /* __VEHICLE_CONFIG_H */
