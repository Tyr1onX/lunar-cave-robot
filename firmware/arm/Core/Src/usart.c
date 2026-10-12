#include "usart.h"

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;

void MX_UART5_Init(void)
{
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 100000;
  huart5.Init.WordLength = UART_WORDLENGTH_9B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_EVEN;
  huart5.Init.Mode = UART_MODE_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uart_handle)
{
  GPIO_InitTypeDef gpio_init = {0};

  if (uart_handle->Instance != UART5)
  {
    return;
  }

  __HAL_RCC_UART5_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio_init.Pin = GPIO_PIN_2;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  /* 上拉：接收机未供电/未插时把 RX 线拉成空闲高电平，避免 UART 收到连续“假起始位”
     产生帧错误（冷启动时正好会打到 UART5 的 DMA 接收，见 Remote_Supervise 注释）。 */
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOD, &gpio_init);

  hdma_uart5_rx.Instance = DMA1_Stream1;
  hdma_uart5_rx.Init.Request = DMA_REQUEST_UART5_RX;
  hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart5_rx.Init.Mode = DMA_NORMAL;
  hdma_uart5_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(uart_handle, hdmarx, hdma_uart5_rx);

  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uart_handle)
{
  if (uart_handle->Instance != UART5)
  {
    return;
  }

  __HAL_RCC_UART5_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
  HAL_DMA_DeInit(uart_handle->hdmarx);
  HAL_NVIC_DisableIRQ(UART5_IRQn);
}
