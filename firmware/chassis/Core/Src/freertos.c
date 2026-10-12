/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* 说明 ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* 说明 --------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "fdcan.h"
#include "usart.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "kinematics.h"
#include "vehicle_config.h"
#include "hm_motor.h"
#include "rc_sbus.h"
#include "chassis_command.h"
#include "chassis_safety.h"
#include "Chaiss_Communication.h"
#include <math.h>

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
/* USER CODE END Includes */

/* 说明 ------------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* 说明 --------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* 说明 ------------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* 说明 ----------------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId INS_TASKHandle;
osThreadId CHASSISR_TASKHandle;

/* 说明 ------------------------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* 一阶低通滤波：alpha 取 0~1，值越大响应越快，滤波效果越弱。 */
static inline float lpf_first_order(float prev, float input, float alpha)
{
  if (alpha < 0.0f) alpha = 0.0f;
  else if (alpha > 1.0f) alpha = 1.0f;
  return prev + alpha * (input - prev);
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void INS_Task(void const * argument);
void ChassisR_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  说明 FreeRTOS
  * @param  ?
  * @retval ?
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 说明 */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 说明 */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 说明 */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 说明 */
  /* USER CODE END RTOS_QUEUES */

  /* 说明 */
  /* 说明 */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* 说明 */
  osThreadDef(INS_TASK, INS_Task, osPriorityRealtime, 0, 512);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* 说明 */
  osThreadDef(CHASSISR_TASK, ChassisR_Task, osPriorityAboveNormal, 0, 512);
  CHASSISR_TASKHandle = osThreadCreate(osThread(CHASSISR_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 说明 */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  说明
  * @param  argument说明
  * @retval ?
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /*
   * 遥控任务只生成“目标值”，不直接发送电机命令：
   * CH8 三段拨杆选择控制源：下拨失能，中置仅串口，上拨仅 SBUS。
   * SBUS 遥控器模拟 Twist 指令：
   *   CH3 -> linear_x (m/s)
   *   CH1 -> angular_z (rad/s)
   * 底盘任务只读取统一 chassis_command，不关心指令来源。
   */
  static rc_control_mode_e active_control_mode = RC_CONTROL_DISABLED;
  static chassis_safety_t chassis_safety;
  static uint8_t chassis_safety_initialized = 0u;
  static float yaw_rate_cmd_filt = 0.0f;
  static float v_cmd_filt = 0.0f;
  static uint32_t last_uart_error_count = 0u;
  const float yaw_lpf_alpha = 0.2f;
  const float v_lpf_alpha   = 0.2f;

  /* 说明 */
  for(;;)
  {
    if (chassis_safety_initialized == 0u)
    {
      chassis_safety_init(&chassis_safety);
      chassis_safety_initialized = 1u;
    }
    rc_sbus_update();

    {
      rc_control_mode_e requested_mode = RC_CONTROL_DISABLED;
      uint8_t rc_neutral;
      uint8_t serial_fresh;
      uint8_t serial_neutral;
      uint8_t uart_fault_event;
      chassis_safety_state_e safety_state;

      if (rc_sbus.online)
      {
        requested_mode = rc_control_mode_from_pwm(
            rc_sbus.ch_pwm[g_sbus_ch_map.control_mode]);
      }

      rc_neutral =
          (rc_sbus_channel_norm(g_sbus_ch_map.throttle) == 0.0f &&
           rc_sbus_channel_norm(g_sbus_ch_map.steer) == 0.0f) ? 1u : 0u;

      /*
       * Keep parsing serial input while unarmed only in the explicitly selected
       * serial mode. The independent chassis command gate still rejects motion.
       */
      Chaiss_Communication_SetCommandEnabled(
          (rc_sbus.online && requested_mode == RC_CONTROL_SERIAL) ? 1u : 0u);
      serial_fresh = Chaiss_Communication_IsOnline();
      serial_neutral =
          (fabsf(g_chaiss_communication.vx) <= 0.0001f &&
           fabsf(g_chaiss_communication.vy) <= 0.0001f &&
           fabsf(g_chaiss_communication.vz) <= 0.0001f) ? 1u : 0u;
      uart_fault_event =
          (g_chaiss_communication.uart_error_count != last_uart_error_count) ? 1u : 0u;
      last_uart_error_count = g_chaiss_communication.uart_error_count;
      safety_state = chassis_safety_update(
          &chassis_safety, rc_sbus.online ? 1u : 0u, requested_mode,
          rc_neutral, serial_fresh, serial_neutral, uart_fault_event);
      chassis_command_set_safety_enabled(
          (safety_state == CHASSIS_SAFETY_ARMED) ? 1u : 0u);

      if (requested_mode != active_control_mode)
      {
        /* 切换控制源时先清掉旧指令，避免旧命令跨模式残留。 */
        chassis_command_stop(CHASSIS_CMD_RC);
        active_control_mode = requested_mode;
        yaw_rate_cmd_filt = 0.0f;
        v_cmd_filt = 0.0f;
      }

      if ((requested_mode == RC_CONTROL_DISABLED) ||
          (safety_state != CHASSIS_SAFETY_ARMED))
      {
        /*
         * Boot, receiver recovery, mode changes and serial lease expiry all
         * remain logically disarmed until CH8 is deliberately cycled through
         * disabled with neutral controls. This software gate is not an E-stop.
         */
        chassis_command_stop(CHASSIS_CMD_RC);
        osDelay(10);
        continue;
      }

      if (requested_mode == RC_CONTROL_SERIAL)
      {
        /* CH8 中置：仅在新鲜零速帧完成手动互锁后允许串口控制。 */
        osDelay(10);
        continue;
      }

      /* CH8 上拨：关闭 USART1 指令入口，只发布 SBUS 遥控指令。 */
      Chaiss_Communication_SetCommandEnabled(0u);

      /* CH1 -> angular_z；不再积分为目标航向角。 */
      {
        /*
         * 正值约定为逆时针。IMU 反馈闭环在 Chassis_task 中完成。
         */
        float yaw_rate_raw = rc_sbus_channel_norm(g_sbus_ch_map.steer)
                           * YAW_RATE_MAX_RADPS * RC_STEER_SIGN;
        yaw_rate_cmd_filt = lpf_first_order(yaw_rate_cmd_filt, yaw_rate_raw, yaw_lpf_alpha);
      }

      /* CH3 -> 车体系线速度指令。 */
      {
        /* 油门归一化范围约为 [-1,+1]，再换算为车辆线速度 m/s。 */
        float v_set_raw = rc_sbus_channel_norm(g_sbus_ch_map.throttle) * V_CMD_MAX_MS;
        v_cmd_filt = lpf_first_order(v_cmd_filt, v_set_raw, v_lpf_alpha);
        chassis_move.v_set = v_cmd_filt;
      }

      if (rc_sbus.ch_pwm[g_sbus_ch_map.heading_lock] <
          SBUS_HEADING_LOCK_THRESHOLD)
      {
        /* SWC up: capture and hold the current yaw angle. */
        chassis_command_publish_heading_lock(CHASSIS_CMD_RC);
      }
      else
      {
        /* SWC down: leave heading lock and restore normal RC velocity mode. */
        chassis_command_publish(v_cmd_filt, yaw_rate_cmd_filt,
                                1u, CHASSIS_CMD_RC);
      }
    }

    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief 说明
* @param argument说明
* @retval ?
*/
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */

  /* 说明 */
  for(;;)
  {
    INS_task();		
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_ChassisR_Task */
/**
* @brief Function implementing the CHASSISR_TASK thread.
* @param argument说明
* @retval ?
*/
/* USER CODE END Header_ChassisR_Task */
void ChassisR_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisR_Task */
  /* 说明 */
  for(;;)
  {
    Chassis_task();
  }
  /* USER CODE END ChassisR_Task */
}

/* 私有应用代码 --------------------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


