#include "can_bsp.h"
#include "fdcan.h"
#include "string.h"
#include "hm_motor.h"
#include "chassis_task.h"
#include "vehicle_config.h"

/*
 * 学生阅读提示：
 * CAN1 只连接左侧 1/3/5 号电机，反馈进入 FIFO0；
 * CAN2 只连接右侧 2/4/6 号电机，反馈进入 FIFO1。
 * 六台电机的反馈 ID 已配置为 0x10+命令ID，因此必须用报文头 Identifier 分发。
 */

FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* 配置接收滤波器。 */	
	sFilterConfig.IdType = FDCAN_STANDARD_ID;//扩展ID不接收
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; // 
  sFilterConfig.FilterID2 = 0x00000000; // 
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
/* 全局过滤设置 */
/* 接收到消息ID与标准ID过滤不匹配，不接受 */
/* 接收到消息ID与扩展ID过滤不匹配，不接受 */
/* 过滤标准ID远程帧 */ 
/* 过滤扩展ID远程帧 */ 
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

	/* 开启RX FIFO0的新数据中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 

  /* 启动 FDCAN 外设。 */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* 配置接收滤波器。 */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* 配置全局滤波器：拒绝标准/扩展远程帧以及不匹配的报文。 */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* 开启 FDCAN2 的接收 FIFO1 新消息中断。 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;                 // CAN ID
  TxHeader.IdType =  FDCAN_STANDARD_ID ;        
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
  if(len<=8)	
	{
	  TxHeader.DataLength = len;     // 发送长度：8byte
	}
	else  if(len==12)	
	{
	   TxHeader.DataLength =FDCAN_DLC_BYTES_12;
	}
	else  if(len==16)	
	{
	  TxHeader.DataLength =FDCAN_DLC_BYTES_16;
	
	}
  else  if(len==20)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_20;
	}		
	else  if(len==24)	
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_24;	
	}else  if(len==48)
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_48;
	}else  if(len==64)
   {
		 TxHeader.DataLength =FDCAN_DLC_BYTES_64;
	 }
											
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//比特率切换关闭，不适用于经典CAN
  TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            // 使用经典 CAN 帧，而不是 CAN FD
  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
  TxHeader.MessageMarker = 0;//消息标记

    /* 带重试的发送，最多尝试5次 */
    {
        volatile uint32_t retry = 5u;
        HAL_StatusTypeDef can_sta;
        do
        {
            can_sta = HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data);
            if (can_sta == HAL_OK) break;
            __NOP(); __NOP(); __NOP();
        } while (--retry > 0u);
    }
    return 0;
}


/* G36 电机反馈分发：CAN ID 匹配法
 * 每个电机回复帧使用唯一 CAN ID = 0x10 + 命令ID（如 ID1 回复 0x11），
 * 通过 RxHeader.Identifier 即可确定来源电机。
 * data[0] 仍保持电机 ID，但不再用于匹配，只传给解码函数。 */
static void dispatch_motor_feedback(motor_can_bus_e bus, uint16_t rx_id, const uint8_t *data, uint32_t len)
{
    unsigned char i;

    if (len < 8u)
    {
        return;
    }

    /*
     * 同时检查总线和反馈 ID，避免接线错误时把右侧反馈写进左侧电机对象。
     * 例如左后命令 ID=5，正确反馈 ID=0x15，并且必须来自 CAN1。
     */
    for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
    {
        if (g_wheel_motor_config[i].bus == bus &&
            rx_id == (0x10u + g_wheel_motor_config[i].can_id))
        {
            hm_motor_decode_feedback(&chassis_move.motor[i], data, 8u, HAL_GetTick());
            break;
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            memset(g_Can1RxData, 0, sizeof(g_Can1RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
            /* 左侧 3 个 G36 电机的反馈 */
            dispatch_motor_feedback(MOTOR_CAN1, RxHeader1.Identifier, g_Can1RxData, 8u);
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN2)
    {
      /* Retrieve Rx messages from RX FIFO1 */
			memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
      /* 右侧 3 个 G36 电机的反馈 */
      dispatch_motor_feedback(MOTOR_CAN2, RxHeader2.Identifier, g_Can2RxData, 8u);
    }
  }
}
