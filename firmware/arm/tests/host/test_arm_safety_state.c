#include "arm_safety_state.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static arm_safety_inputs_t ready_inputs(void)
{
    arm_safety_inputs_t inputs;
    memset(&inputs, 0, sizeof(inputs));
    inputs.rc_online = 1u;
    inputs.feedback_initialized = 1u;
    return inputs;
}

int main(void)
{
    arm_safety_state_t safety;
    arm_safety_inputs_t inputs = ready_inputs();
    arm_safety_state_init(&safety);
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_BOOT_LOCKED);

    /* RC recovery by itself never resumes the previous target. */
    inputs.host_control = 1u;
    inputs.host_command_fresh = 1u;
    inputs.host_disabled_seen = 1u;
    inputs.host_enable_after_disabled = 1u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_BOOT_LOCKED);

    /* A physical reset sequence can select manual RC control after feedback initializes. */
    inputs.host_control = 0u;
    inputs.manual_rearm_action = 1u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_ARMED);
    inputs.manual_rearm_action = 0u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_ARMED);
    inputs.host_control = 1u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_FAULT_LATCHED);

    /* Host mode requires a new physical action and a valid disabled -> enabled edge. */
    arm_safety_state_init(&safety);
    inputs = ready_inputs();
    inputs.host_control = 1u;
    inputs.host_command_fresh = 1u;
    inputs.host_disabled_seen = 1u;
    inputs.host_enable_after_disabled = 1u;
    inputs.manual_rearm_action = 1u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_ARMED);

    /* A host disable frame, lease expiry, or host-control mode loss latches. */
    inputs.host_disable_event = 1u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_FAULT_LATCHED);
    inputs.host_disable_event = 0u;
    inputs.manual_rearm_action = 1u;
    inputs.host_disabled_seen = 0u;
    inputs.host_enable_after_disabled = 0u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_FAULT_LATCHED);
    inputs.host_disabled_seen = 1u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_FAULT_LATCHED);
    inputs.host_enable_after_disabled = 1u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_ARMED);

    inputs.manual_rearm_action = 0u;
    inputs.host_command_fresh = 0u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_FAULT_LATCHED);
    inputs.host_command_fresh = 1u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_FAULT_LATCHED);

    inputs.manual_rearm_action = 1u;
    inputs.host_control = 0u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_ARMED);
    inputs.host_control = 1u;
    inputs.manual_rearm_action = 0u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_FAULT_LATCHED);

    /* An offline RC controller and uninitialized feedback both block re-arm. */
    inputs.rc_online = 0u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_FAULT_LATCHED);
    inputs.rc_online = 1u;
    inputs.manual_rearm_action = 1u;
    inputs.feedback_initialized = 0u;
    assert(arm_safety_state_update(&safety, &inputs) == ARM_SAFETY_FAULT_LATCHED);

    puts("PASS arm firmware safety-state host tests");
    return 0;
}
