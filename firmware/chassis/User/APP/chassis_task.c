/**
 ******************************************************************************
 * @file    chassis_task.c
 * @brief   6 轮差速移动车底盘任务实现（详见 chassis_task.h）
 ******************************************************************************
 */

#include "chassis_task.h"
#include "kinematics.h"
#include "fdcan.h"
#include "cmsis_os.h"

extern INS_t INS;

chassis_t chassis_move;

static float clampf(float value, float limit)
{
    /* 对称限幅：把 value 限制在 [-limit, +limit]。 */
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

static float ramp_to(float current, float target, float max_step)
{
    float delta = target - current;
    if (delta > max_step)
    {
        delta = max_step;
    }
    else if (delta < -max_step)
    {
        delta = -max_step;
    }
    return current + delta;
}

/* 按电机配置表选择所在的 CAN 总线句柄 */
static hcan_t *bus_handle(motor_can_bus_e bus)
{
    return (bus == MOTOR_CAN1) ? (hcan_t *)&hfdcan1 : (hcan_t *)&hfdcan2;
}

void Chassis_init(chassis_t *chassis)
{
    unsigned char i;
    unsigned char j;

    /* 数组顺序必须与 vehicle_config.h 中 wheel_index_e 完全一致。 */
    for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
    {
        chassis->motor[i].id = g_wheel_motor_config[i].can_id;
    }

    /* 上电后连续发送使能帧，保证 6 个电机可靠进入 MIT 模式 */
    for (j = 0u; j < 5u; j++)
    {
        for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
        {
            hm_motor_enable(bus_handle(g_wheel_motor_config[i].bus),
                            g_wheel_motor_config[i].can_id);
            osDelay(1);
        }
    }
}

void Chassis_task(void)
{
    float motor_vel[WHEEL_COUNT];
    float motor_speed[WHEEL_COUNT];
    float v_applied = 0.0f; /* 经过斜坡限制后的线速度指令 */
    float yaw_rate_applied = 0.0f;
    float yaw_rate_imu_filt = 0.0f;
    float yaw_rate_integral = 0.0f;
    const float dt = (float)CHASSIS_TASK_PERIOD_MS / 1000.0f;
    unsigned char i;

    /* 等待 IMU 收敛 */
    while (INS.ins_flag == 0u)
    {
        osDelay(1);
    }

    Chassis_init(&chassis_move);

    while (1)
    {
        /* 第 1 步：读取 IMU。总航向角不会在 +/-PI 处跳变。 */
        chassis_move.yaw   = INS.YawTotalAngle;
        chassis_move.d_yaw = INS.Gyro[2];

        /* 第 2 步：六个电机反馈经轮径和安装方向换算为车体速度。 */
        for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
        {
            motor_vel[i] = chassis_move.motor[i].vel;
        }
        kinematics_feedback_6wd(motor_vel,
                                &chassis_move.v_left, &chassis_move.v_right,
                                &chassis_move.v, &chassis_move.omega);

        {
            chassis_command_t command;
            uint8_t command_ok = chassis_command_read(&command);

            if ((command_ok != 0u) &&
                (command.enabled != 0u) &&
                (chassis_command_is_fresh(&command, HAL_GetTick(),
                                          CHASSIS_CMD_TIMEOUT_MS) != 0u))
            {
                chassis_move.start_flag = 1u;
                chassis_move.command_source = command.source;

                if (command.mode == CHASSIS_MODE_HEADING_LOCK)
                {
                    /* Capture the current continuous yaw only on mode entry. */
                    if (chassis_move.heading_lock_active == 0u)
                    {
                        chassis_move.heading_lock_target = chassis_move.yaw;
                    }
                    chassis_move.heading_lock_active = 1u;
                    chassis_move.heading_lock_error =
                        chassis_move.heading_lock_target - chassis_move.yaw;
                    chassis_move.v_set = 0.0f;
                    chassis_move.yaw_rate_set = clampf(
                        HEADING_LOCK_KP * chassis_move.heading_lock_error,
                        HEADING_LOCK_RETURN_RATE_MAX_RADPS);
                }
                else
                {
                    chassis_move.heading_lock_active = 0u;
                    chassis_move.heading_lock_error = 0.0f;
                    chassis_move.v_set =
                        clampf(command.linear_x, V_CMD_MAX_MS);
                    chassis_move.yaw_rate_set =
                        clampf(command.angular_z, YAW_RATE_MAX_RADPS);
                }
            }
            else
            {
                chassis_move.start_flag = 0u;
                chassis_move.v_set = 0.0f;
                chassis_move.yaw_rate_set = 0.0f;
                chassis_move.heading_lock_active = 0u;
                chassis_move.heading_lock_error = 0.0f;
                chassis_move.command_source = CHASSIS_CMD_NONE;
            }
        }

        if (chassis_move.start_flag != 0u)
        {
            float yaw_rate_raw;
            float yaw_rate_correction;

            /*
             * 第 3 步：角速度运动学前馈 + IMU 反馈。
             * 前馈保证在误差为零时仍能维持目标转速；PI 只补偿打滑、
             * 左右负载差和模型误差。KI 默认关闭，先在低速下调 KP。
             */
            yaw_rate_raw = INS.Gyro[2] * IMU_YAW_RATE_SIGN;
            yaw_rate_imu_filt += IMU_YAW_RATE_LPF_ALPHA *
                                 (yaw_rate_raw - yaw_rate_imu_filt);
            chassis_move.d_yaw = yaw_rate_imu_filt;

            v_applied = ramp_to(v_applied, chassis_move.v_set,
                                V_CMD_RAMP_MPS2 * dt);
            yaw_rate_applied = ramp_to(yaw_rate_applied,
                                       chassis_move.yaw_rate_set,
                                       YAW_RATE_RAMP_RADPS2 * dt);

            chassis_move.yaw_rate_error =
                yaw_rate_applied - yaw_rate_imu_filt;
            yaw_rate_integral += chassis_move.yaw_rate_error * dt;
            yaw_rate_integral = clampf(yaw_rate_integral,
                                       YAW_RATE_INTEGRAL_LIMIT);
            yaw_rate_correction =
                YAW_RATE_KP * chassis_move.yaw_rate_error +
                YAW_RATE_KI * yaw_rate_integral;
            yaw_rate_correction =
                clampf(yaw_rate_correction,
                       YAW_RATE_CORRECTION_MAX_RADPS);
            chassis_move.yaw_rate_control =
                yaw_rate_applied + yaw_rate_correction;

            /*
             * kinematics_inverse uses:
             *   left=v+turn, right=v-turn.
             * Positive yaw needs right>left, hence the minus sign here.
             */
            chassis_move.turn_out =
                -0.5f * VEHICLE_TRACK_WIDTH_M *
                chassis_move.yaw_rate_control;

            /*
             * 第 4 步：运动学分配。先求左右侧线速度，再除以轮半径得到
             * 电机 rad/s，最后乘每台电机的安装方向 dir。
             */
            kinematics_setpoints_6wd(v_applied, chassis_move.turn_out, motor_speed);

            for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
            {
                hm_motor_send_velocity(bus_handle(g_wheel_motor_config[i].bus),
                                       g_wheel_motor_config[i].can_id,
                                       motor_speed[i]);
            }
        }
        else
        {
            /*
             * 未使能或指令超时时发送全零 MIT 指令让电机放松，并清除
             * 速度斜坡及 PI 状态，避免重新接管时突然运动。
             */
            v_applied = 0.0f;
            yaw_rate_applied = 0.0f;
            yaw_rate_integral = 0.0f;
            chassis_move.turn_out = 0.0f;
            chassis_move.yaw_rate_error = 0.0f;
            chassis_move.yaw_rate_control = 0.0f;
            chassis_move.v_set = 0.0f;
            chassis_move.yaw_rate_set = 0.0f;

            for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
            {
                hm_motor_send_relax(bus_handle(g_wheel_motor_config[i].bus),
                                    g_wheel_motor_config[i].can_id);
            }
        }

        osDelay(CHASSIS_TASK_PERIOD_MS);
    }
}
