#include "bsp_can.hpp"


/* CAN 滤波器配置 */
void Bsp_CAN::FilterConfig(const bool is_extended_) const
{
	const FDCAN_FilterTypeDef filter_cfg_t {is_extended_ ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
		0, FDCAN_FILTER_MASK, FDCAN_FILTER_TO_RXFIFO0,
		0x00000000, 0x00000000, 0, 0};
	if (HAL_FDCAN_ConfigFilter(handle, &filter_cfg_t) != HAL_OK) Error_Handler();
	HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT,
		FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(handle, FDCAN_CFG_RX_FIFO0, 1);
}

/* 发送一帧；发送队列满时直接丢弃（不阻塞控制任务） */
bool Bsp_CAN::Transmit(const uint32_t id_, const uint8_t* data_, const uint8_t length_)
{
	tx_header.Identifier = id_;
	tx_header.DataLength = length_;
	if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &tx_header, data_) != HAL_OK)
	{
		++tx_failure_count;
		return false;
	}
	return true;
}

bool Bsp_CAN::IsBusOff() const
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	const bool bus_off = error.bus_off != 0U;
	__set_PRIMASK(primask);
	return bus_off;
}

uint32_t Bsp_CAN::TxFailureCount() const
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	const uint32_t failures = tx_failure_count;
	__set_PRIMASK(primask);
	return failures;
}

uint32_t Bsp_CAN::RxFailureCount() const
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	const uint32_t failures = rx_failure_count;
	__set_PRIMASK(primask);
	return failures;
}

uint32_t Bsp_CAN::ErrorEventCount() const
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	const uint32_t events = error_count;
	__set_PRIMASK(primask);
	return events;
}

can_health_snapshot_s Bsp_CAN::HealthSnapshot() const
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	const can_health_snapshot_s snapshot
	{
		error.bus_off,
		tx_failure_count,
		rx_failure_count,
		error_count
	};
	__set_PRIMASK(primask);
	return snapshot;
}
