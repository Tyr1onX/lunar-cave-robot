#include "chassis_safety.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    chassis_safety_t safety;
    chassis_safety_init(&safety);
    assert(safety.state == CHASSIS_SAFETY_BOOT_LOCKED);

    /* A selected drive mode at power-up cannot arm the controller. */
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SBUS, 1u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_BOOT_LOCKED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_DISABLED, 1u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_DISARMED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SBUS, 0u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_DISARMED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SBUS, 1u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_ARMED);

    /* Receiver loss latches; link recovery with the switch still high is insufficient. */
    assert(chassis_safety_update(&safety, 0u, RC_CONTROL_DISABLED, 1u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_FAULT_LATCHED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SBUS, 1u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_FAULT_LATCHED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_DISABLED, 1u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_DISARMED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SBUS, 1u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_ARMED);

    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SBUS, 1u, 0u, 0u, 1u) ==
           CHASSIS_SAFETY_FAULT_LATCHED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SBUS, 1u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_FAULT_LATCHED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_DISABLED, 1u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_DISARMED);

    /* An active serial lease expiry requires the same operator reset sequence. */
    chassis_safety_init(&safety);
    (void)chassis_safety_update(&safety, 1u, RC_CONTROL_DISABLED, 1u, 0u, 0u, 0u);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SERIAL, 1u, 1u, 1u, 0u) ==
           CHASSIS_SAFETY_ARMED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SERIAL, 1u, 0u, 1u, 0u) ==
           CHASSIS_SAFETY_FAULT_LATCHED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SERIAL, 1u, 1u, 1u, 0u) ==
           CHASSIS_SAFETY_FAULT_LATCHED);
    (void)chassis_safety_update(&safety, 1u, RC_CONTROL_DISABLED, 1u, 0u, 0u, 0u);
    /* The neutral selector position followed by an active neutral command is
     * the deliberate manual reset/re-arm sequence. */
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SERIAL, 1u, 1u, 1u, 0u) ==
           CHASSIS_SAFETY_ARMED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SERIAL, 1u, 1u, 1u, 0u) ==
           CHASSIS_SAFETY_ARMED);

    /* Switching from the armed source latches; a source change cannot re-arm by itself. */
    chassis_safety_init(&safety);
    (void)chassis_safety_update(&safety, 1u, RC_CONTROL_DISABLED, 1u, 0u, 0u, 0u);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SBUS, 1u, 0u, 0u, 0u) ==
           CHASSIS_SAFETY_ARMED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SERIAL, 1u, 1u, 1u, 0u) ==
           CHASSIS_SAFETY_FAULT_LATCHED);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SERIAL, 1u, 1u, 1u, 0u) ==
           CHASSIS_SAFETY_FAULT_LATCHED);
    (void)chassis_safety_update(&safety, 1u, RC_CONTROL_DISABLED, 1u, 0u, 0u, 0u);
    assert(chassis_safety_update(&safety, 1u, RC_CONTROL_SERIAL, 1u, 1u, 1u, 0u) ==
           CHASSIS_SAFETY_ARMED);

    puts("PASS chassis firmware safety-state host tests");
    return 0;
}
