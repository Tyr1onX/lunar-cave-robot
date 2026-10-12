#include <assert.h>
#include <stdint.h>

#include "rc_control_mode.h"

int main(void)
{
    assert(rc_control_mode_from_pwm(1200u) == RC_CONTROL_DISABLED);
    assert(rc_control_mode_from_pwm(1500u) == RC_CONTROL_SERIAL);
    assert(rc_control_mode_from_pwm(1800u) == RC_CONTROL_SBUS);
    return 0;
}
