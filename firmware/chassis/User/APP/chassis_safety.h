#ifndef __CHASSIS_SAFETY_H
#define __CHASSIS_SAFETY_H

#include <stdint.h>
#include "rc_control_mode.h"

typedef enum
{
    CHASSIS_SAFETY_BOOT_LOCKED = 0,
    CHASSIS_SAFETY_DISARMED,
    CHASSIS_SAFETY_ARMED,
    CHASSIS_SAFETY_FAULT_LATCHED
} chassis_safety_state_e;

typedef struct
{
    chassis_safety_state_e state;
    rc_control_mode_e armed_mode;
    uint8_t disabled_position_seen;
} chassis_safety_t;

void chassis_safety_init(chassis_safety_t *safety);
chassis_safety_state_e chassis_safety_update(
    chassis_safety_t *safety,
    uint8_t receiver_online,
    rc_control_mode_e requested_mode,
    uint8_t rc_controls_neutral,
    uint8_t serial_command_fresh,
    uint8_t serial_command_neutral,
    uint8_t link_fault_event);

#endif
