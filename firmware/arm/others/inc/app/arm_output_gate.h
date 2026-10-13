#ifndef ARM_OUTPUT_GATE_H
#define ARM_OUTPUT_GATE_H

#include <stdint.h>

#define ARM_OUTPUT_MOTION 0u
#define ARM_OUTPUT_RECOVERY 1u

#ifdef __cplusplus
extern "C" {
#endif

/* Output begin holds the FreeRTOS critical section until arm_output_gate_end(). */
uint8_t arm_output_gate_begin(uint8_t output_kind);
void arm_output_gate_end(void);
void arm_output_gate_revoke(void);
void arm_output_gate_trip(void);
uint8_t arm_output_gate_begin_resume(uint8_t host_control, uint32_t remote_mode_generation,
	uint32_t can_tx_failure_count, uint32_t can_rx_failure_count,
	uint32_t can_error_event_count);
uint8_t arm_output_gate_activate(void);
uint8_t arm_output_gate_authorization_current(void);

#ifdef __cplusplus
}
#endif

#endif
