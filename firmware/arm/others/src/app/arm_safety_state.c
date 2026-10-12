#include "arm_safety_state.h"

void arm_safety_state_init(arm_safety_state_t *safety)
{
    if (safety == 0)
    {
        return;
    }
    safety->state = ARM_SAFETY_BOOT_LOCKED;
    safety->manual_rearm_seen = 0u;
    safety->armed_host_control = 0u;
}

arm_safety_state_e arm_safety_state_update(
    arm_safety_state_t *safety, const arm_safety_inputs_t *inputs)
{
    if ((safety == 0) || (inputs == 0))
    {
        return ARM_SAFETY_FAULT_LATCHED;
    }

    if ((inputs->rc_online == 0u) || (inputs->hold_requested != 0u))
    {
        safety->state = ARM_SAFETY_FAULT_LATCHED;
        safety->manual_rearm_seen = 0u;
        return safety->state;
    }

    if (safety->state == ARM_SAFETY_ARMED)
    {
        if ((safety->armed_host_control != inputs->host_control) ||
            ((safety->armed_host_control != 0u) &&
             ((inputs->host_disable_event != 0u) ||
              (inputs->host_command_fresh == 0u))))
        {
            safety->state = ARM_SAFETY_FAULT_LATCHED;
            safety->manual_rearm_seen = 0u;
            safety->armed_host_control = 0u;
        }
        return safety->state;
    }

    if (inputs->manual_rearm_action != 0u)
    {
        safety->manual_rearm_seen = 1u;
    }

    if ((safety->manual_rearm_seen == 0u) ||
        (inputs->feedback_initialized == 0u))
    {
        return safety->state;
    }

    /* Manual RC mode needs an explicit physical re-arm; host mode also needs
     * a fresh disabled -> enabled command handshake and a current accepted frame. */
    if ((inputs->host_control == 0u) ||
        ((inputs->host_disabled_seen != 0u) &&
         (inputs->host_enable_after_disabled != 0u) &&
         (inputs->host_command_fresh != 0u)))
    {
        safety->state = ARM_SAFETY_ARMED;
        safety->armed_host_control = inputs->host_control;
    }
    return safety->state;
}
