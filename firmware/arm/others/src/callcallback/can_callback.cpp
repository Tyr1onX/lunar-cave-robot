#include "roboarm.hpp"


extern Bsp_CAN abc_can3;

extern Joint j1_left;
extern Joint j2_left;
extern Joint j3_left;
extern Joint j4_left;
extern Joint j5_left;
extern Joint j6_left;
extern Joint j7_left;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	UNUSED(RxFifo0ITs);
	if (hfdcan == abc_can3.handle)  // friendly
	{
		abc_can3.Receive();
		switch (abc_can3.rx_header.Identifier)
		{
		case 0x11:
			j1_left.UpdateFeedback(abc_can3.rx_buffer);
			break;
		case 0x12:
			j2_left.UpdateFeedback(abc_can3.rx_buffer);
			break;
		case 0x13:
			j3_left.UpdateFeedback(abc_can3.rx_buffer);
			break;
		case 0x14:
			j4_left.UpdateFeedback(abc_can3.rx_buffer);
			break;
		case 0x15:
			j5_left.UpdateFeedback(abc_can3.rx_buffer);
			break;
		case 0x16:
			j6_left.UpdateFeedback(abc_can3.rx_buffer);
			break;
		case 0x17:
			j7_left.UpdateFeedback(abc_can3.rx_buffer);
			break;
		default:
			break;
		}
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if (hfdcan == abc_can3.handle)
	{
		FDCAN_ErrorCountersTypeDef error_counters {};
		FDCAN_ProtocolStatusTypeDef protocol_status {};
		HAL_FDCAN_GetErrorCounters(hfdcan, &error_counters);
		HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status);
		abc_can3.error.hcan = hfdcan;
		abc_can3.error.err_bits = ErrorStatusITs;
		abc_can3.error.hal_error = HAL_FDCAN_GetError(hfdcan);
		abc_can3.error.last_protocol_error = protocol_status.LastErrorCode;
		abc_can3.error.tx_error_cnt = error_counters.TxErrorCnt;
		abc_can3.error.rx_error_cnt = error_counters.RxErrorCnt;
		abc_can3.error.bus_off = protocol_status.BusOff;
		++abc_can3.error_count;
	}
}
