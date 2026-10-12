/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 六轮差速车主程序入口
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
/* 头文件 ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* 用户头文件 --------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088driver.h"
#include "kalman_filter.h"
#include "mahony_filter.h"
#include "bsp_dwt.h"
#include "BMI088Middleware.h"
#include "can_bsp.h"
#include "rc_sbus.h"
#include "Chaiss_Communication.h"
/* USER CODE END Includes */

/* 用户类型定义 ------------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* 用户宏定义 --------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* 用户宏 ------------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* 用户变量 ----------------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* 函数声明 ------------------------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* 用户私有代码 -------------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  应用程序入口
  * @retval 程序返回值；嵌入式程序正常运行时不会返回
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* 微控制器配置 -------------------------------------------------------------*/

  /* 复位所有外设，初始化 Flash 接口和 HAL 系统时基。 */
  HAL_Init();

  /* USER CODE BEGIN Init */
//06 01 00 00 00 00 00 00 00 00 00 00 00 00 80 3F
  /* USER CODE END Init */

  /* 配置系统主时钟。 */
  SystemClock_Config();

  /* 配置 FDCAN 等外设使用的公共时钟。 */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* 初始化 CubeMX 配置的全部外设。 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM3_Init();
  MX_FDCAN3_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  	DWT_Init(480);
  
  /* 初始化 BMI088；初始化失败时持续重试，防止在 IMU 不可用时启动底盘。 */
  while (BMI088_init(&hspi2, 0) != BMI088_NO_ERROR)
	{
	  ;
	}
	
	
	
	/* IMU 初始化完成后开启两路外设电源输出。 */
	Power_OUT1_ON;
	Power_OUT2_ON;
	
  /* 配置并启动左右两侧电机使用的两路 FDCAN。 */
  FDCAN1_Config();
	FDCAN2_Config();

  /* UART5 receives SBUS; USART1 (PA9/PA10, 115200 8N1) receives VX/VY/VZ. */
  rc_sbus_init();
  Chaiss_Communication_Init();  /* USER CODE END 2 */

  /* 调用 FreeRTOS 对象初始化函数。 */
  MX_FREERTOS_Init();

  /* 启动 FreeRTOS 调度器；成功后程序由各任务接管。 */
  osKernelStart();

  /* 正常情况下不会执行到下面的 while 循环。 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief 配置系统时钟
  * @retval 无
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** 允许更新电源配置
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** 配置主内部稳压器输出电压
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** 根据指定参数初始化 RCC 振荡器
  * 参数保存在 RCC_OscInitTypeDef 结构体中。
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** 初始化 CPU、AHB 和 APB 总线时钟
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief 配置外设公共时钟
  * @retval 无
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** 初始化外设时钟
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 24;
  PeriphClkInitStruct.PLL2.PLL2N = 200;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  定时器周期到达回调函数
  * @note   TIM2 产生中断后由 HAL_TIM_IRQHandler() 调用本函数，
  *         再调用 HAL_IncTick() 更新系统时基变量 uwTick。
  * @param  htim 定时器句柄
  * @retval 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  HAL 初始化或配置失败时进入的错误处理函数
  * @retval 无
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 用户可在此添加 HAL 错误状态上报代码。 */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  报告 assert_param 断言失败所在的源文件和行号
  * @param  file 源文件名指针
  * @param  line 发生错误的源代码行号
  * @retval 无
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 可在此输出文件名和行号，例如通过串口打印断言失败位置。 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */





