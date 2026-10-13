#pragma once

#include <stdint.h>
#include "main.h"

typedef struct { uint32_t id; } FDCAN_HandleTypeDef;
typedef struct { uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2, FilterID3, FilterID4; } FDCAN_FilterTypeDef;
typedef struct { uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator, BitrateSwitch, FDFormat, TxEventFifoControl, MessageMarker; } FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t Identifier, DataLength; } FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
typedef struct { uint32_t LastErrorCode, BusOff; } FDCAN_ProtocolStatusTypeDef;
#ifndef HOST_STUB_HAL_STATUS_DEFINED
typedef int32_t HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#endif
#define FDCAN_EXTENDED_ID 1U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_FILTER_MASK 0U
#define FDCAN_FILTER_TO_RXFIFO0 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_ON 1U
#define FDCAN_BRS_OFF 0U
#define FDCAN_FD_CAN 1U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_REJECT 0U
#define FDCAN_REJECT_REMOTE 0U
#define FDCAN_CFG_RX_FIFO0 0U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_BUS_OFF 2U
#define FDCAN_IT_ERROR_PASSIVE 4U
#define FDCAN_IT_ERROR_WARNING 8U

#ifdef __cplusplus
extern "C" {
#endif
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*, const FDCAN_FilterTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigFifoWatermark(FDCAN_HandleTypeDef*, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*, const FDCAN_TxHeaderTypeDef*, const uint8_t*);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*, uint32_t, FDCAN_RxHeaderTypeDef*, uint8_t*);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef*, FDCAN_ErrorCountersTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef*, FDCAN_ProtocolStatusTypeDef*);
uint32_t HAL_FDCAN_GetError(FDCAN_HandleTypeDef*);
void Error_Handler(void);
#ifdef __cplusplus
}
#endif
