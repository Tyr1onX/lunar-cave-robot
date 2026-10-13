#ifndef ARM_SAFETY_STATE_H
#define ARM_SAFETY_STATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ARM_SAFETY_BOOT_LOCKED = 0,
    ARM_SAFETY_FAULT_LATCHED,
    ARM_SAFETY_ARMED
} arm_safety_state_e;

typedef struct
{
    arm_safety_state_e state;
    uint8_t manual_rearm_seen;
    uint8_t armed_host_control;
} arm_safety_state_t;

typedef struct
{
    uint8_t rc_online;
    uint8_t hold_requested;
    uint8_t host_control;
    uint8_t host_command_fresh;
    uint8_t host_disabled_seen;
    uint8_t host_enable_after_disabled;
    uint8_t host_disable_event;
    uint8_t manual_rearm_action;
    uint8_t feedback_healthy;
    uint8_t can_healthy;
} arm_safety_inputs_t;

void arm_safety_state_init(arm_safety_state_t *safety);
void arm_safety_state_latch_fault(arm_safety_state_t *safety);
arm_safety_state_e arm_safety_state_update(
    arm_safety_state_t *safety, const arm_safety_inputs_t *inputs);

#ifdef __cplusplus
}
#endif

#endif
