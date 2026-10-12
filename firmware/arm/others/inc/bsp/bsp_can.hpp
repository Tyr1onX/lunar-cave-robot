#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"

#ifdef __cplusplus
}
#endif


/* CAN 错误现场：由 HAL_FDCAN_ErrorStatusCallback 填充，只记录不处理 */
struct can_err_s
{
	FDCAN_HandleTypeDef* hcan;
	uint32_t err_bits;
	uint32_t hal_error;
	uint32_t last_protocol_error;
	uint32_t tx_error_cnt;
	uint32_t rx_error_cnt;
	uint32_t bus_off;
};

/* FDCAN 收发：只负责帧的搬运与错误现场记录，不含任何业务逻辑。
   always check CubeMX to ensure the FDCAN initialization settings match */
class Bsp_CAN
{
	public:
	explicit Bsp_CAN(FDCAN_HandleTypeDef* hcan_, bool is_extended_ = false,
		bool is_brs_ = false, bool is_fd_ = false)
	: handle(hcan_), tx_header {0, is_extended_ ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
		FDCAN_DATA_FRAME, 8, FDCAN_ESI_ACTIVE,
		is_brs_ ? FDCAN_BRS_ON : FDCAN_BRS_OFF, is_fd_ ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN,
		FDCAN_NO_TX_EVENTS, 0}, error_count(0), error{}
	{}

	void FilterConfig(bool is_extended_ = false) const;

	void Start() const
	{
		if (HAL_FDCAN_Start(handle) != HAL_OK) Error_Handler();
		HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF |
			FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0);
	}

	void Transmit(uint32_t id_, const uint8_t* data_, uint8_t length_ = 8);
	void Receive() { HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &rx_header, rx_buffer); }

	/* 接收与错误回调需要直接读写帧缓冲/错误现场，故声明为友元 */
	friend void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs);
	friend void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs);

	private:
	FDCAN_HandleTypeDef* handle;
	FDCAN_TxHeaderTypeDef tx_header;  // 发送头（接收走 RX FIFO0）
	FDCAN_RxHeaderTypeDef rx_header {};
	uint8_t rx_buffer[8] {};
	uint8_t error_count;  // 进入错误回调的累计次数
	can_err_s error;
};
