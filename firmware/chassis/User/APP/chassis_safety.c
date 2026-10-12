#include "chassis_safety.h"

void chassis_safety_init(chassis_safety_t *safety)
{
    if (safety == 0)
    {
        return;
    }
    safety->state = CHASSIS_SAFETY_BOOT_LOCKED;
    safety->armed_mode = RC_CONTROL_DISABLED;
    safety->disabled_position_seen = 0u;
}

chassis_safety_state_e chassis_safety_update(
    chassis_safety_t *safety,
    uint8_t receiver_online,
    rc_control_mode_e requested_mode,
    uint8_t rc_controls_neutral,
    uint8_t serial_command_fresh,
    uint8_t serial_command_neutral,
    uint8_t link_fault_event)
{
    if (safety == 0)
    {
        return CHASSIS_SAFETY_FAULT_LATCHED;
    }

    if ((receiver_online == 0u) || (link_fault_event != 0u))
    {
        safety->state = CHASSIS_SAFETY_FAULT_LATCHED;
        safety->disabled_position_seen = 0u;
        return safety->state;
    }

    if (requested_mode == RC_CONTROL_DISABLED)
    {
        if (rc_controls_neutral != 0u)
        {
            /* The disabled selector position is the deliberate fault reset step. */
            safety->disabled_position_seen = 1u;
            safety->state = CHASSIS_SAFETY_DISARMED;
            safety->armed_mode = RC_CONTROL_DISABLED;
        }
        else
        {
            safety->state = CHASSIS_SAFETY_DISARMED;
            safety->disabled_position_seen = 0u;
        }
        return safety->state;
    }

    if (safety->state == CHASSIS_SAFETY_ARMED)
    {
        if ((requested_mode != safety->armed_mode) ||
            ((requested_mode == RC_CONTROL_SERIAL) && (serial_command_fresh == 0u)))
        {
            safety->state = CHASSIS_SAFETY_FAULT_LATCHED;
            safety->disabled_position_seen = 0u;
        }
        return safety->state;
    }

    if (safety->state != CHASSIS_SAFETY_DISARMED ||
        safety->disabled_position_seen == 0u)
    {
        return safety->state;
    }

    if (requested_mode == RC_CONTROL_SBUS)
    {
        if (rc_controls_neutral != 0u)
        {
            safety->state = CHASSIS_SAFETY_ARMED;
            safety->armed_mode = requested_mode;
        }
    }
    else if ((requested_mode == RC_CONTROL_SERIAL) &&
             (serial_command_fresh != 0u) && (serial_command_neutral != 0u))
    {
        safety->state = CHASSIS_SAFETY_ARMED;
        safety->armed_mode = requested_mode;
    }

    return safety->state;
}
