#include "rc_control_mode.h"

rc_control_mode_e rc_control_mode_from_pwm(uint16_t pwm)
{
    if (pwm < RC_CONTROL_PWM_DOWN_MAX)
    {
        return RC_CONTROL_DISABLED;
    }

    if (pwm > RC_CONTROL_PWM_UP_MIN)
    {
        return RC_CONTROL_SBUS;
    }

    return RC_CONTROL_SERIAL;
}
