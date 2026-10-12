/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan3;

/* FDCAN3 init function */
void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = ENABLE;
  hfdcan3.Init.TransmitPause = ENABLE;
  hfdcan3.Init.ProtocolException = ENABLE;
  /* f_FDCAN = 120 MHz: 120 MHz / (3 * (1 + 14 + 5)) = 2 Mbps, sample point 75%. */
  hfdcan3.Init.NominalPrescaler = 3;
  hfdcan3.Init.NominalSyncJumpWidth = 10;
  hfdcan3.Init.NominalTimeSeg1 = 29;
  hfdcan3.Init.NominalTimeSeg2 = 10;
  hfdcan3.Init.DataPrescaler = 3;
  hfdcan3.Init.DataSyncJumpWidth = 5;
  hfdcan3.Init.DataTimeSeg1 = 14;
  hfdcan3.Init.DataTimeSeg2 = 5;
  hfdcan3.Init.MessageRAMOffset = 0;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.RxFifo0ElmtsNbr = 6;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */
  /* Regen-proof guard: CubeMX generates TxFifoQueueElmtsNbr = 0 unless it is set in
     CubeMX (Parameter Settings). Without TX FIFO elements the motor command frames
     (HAL_FDCAN_AddMessageToTxFifoQ) would be silently dropped, so re-init with 32. */
  if (hfdcan3.Init.TxFifoQueueElmtsNbr == 0)
  {
    hfdcan3.Init.TxFifoQueueElmtsNbr = 32;
    HAL_FDCAN_DeInit(&hfdcan3);
    if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /* Keep the Message RAM watchdog disabled (WDC = 0) unless deliberate RAM
     handshake supervision is required. A WDI event is then a real fault,
     not an unintended non-default watchdog configuration. */
  if (HAL_FDCAN_ConfigRamWatchdog(&hfdcan3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END FDCAN3_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN3)
  {
  /* USER CODE BEGIN FDCAN3_MspInit 0 */

  /* USER CODE END FDCAN3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN3 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN3 GPIO Configuration
    PD12     ------> FDCAN3_RX
    PD13     ------> FDCAN3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_FDCAN3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN3 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN3_IT1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN3_IT1_IRQn);
  /* USER CODE BEGIN FDCAN3_MspInit 1 */
  /* Fast slew rate for 2 Mbps CAN (CubeMX generates GPIO_SPEED_FREQ_LOW). */
  GPIO_InitTypeDef FDCAN3_FastPins = {0};
  FDCAN3_FastPins.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  FDCAN3_FastPins.Mode = GPIO_MODE_AF_PP;
  FDCAN3_FastPins.Pull = GPIO_NOPULL;
  FDCAN3_FastPins.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  FDCAN3_FastPins.Alternate = GPIO_AF5_FDCAN3;
  HAL_GPIO_Init(GPIOD, &FDCAN3_FastPins);
  /* USER CODE END FDCAN3_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN3)
  {
  /* USER CODE BEGIN FDCAN3_MspDeInit 0 */

  /* USER CODE END FDCAN3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN3 GPIO Configuration
    PD12     ------> FDCAN3_RX
    PD13     ------> FDCAN3_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12|GPIO_PIN_13);

    /* FDCAN3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN3_IT1_IRQn);
  /* USER CODE BEGIN FDCAN3_MspDeInit 1 */

  /* USER CODE END FDCAN3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

