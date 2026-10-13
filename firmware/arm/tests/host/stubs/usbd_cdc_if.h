#pragma once

#include <stdint.h>

#define USBD_OK 0U
#define USBD_STATE_CONFIGURED 3U

typedef struct
{
	uint32_t TxState;
	uint8_t* TxBuffer;
	uint32_t TxLength;
} USBD_CDC_HandleTypeDef;

typedef struct
{
	uint32_t dev_state;
	void* pClassData;
} USBD_HandleTypeDef;

#ifdef __cplusplus
extern "C" {
#endif
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef* device, uint8_t* buffer, uint16_t length);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef* device);
#ifdef __cplusplus
}
#endif
