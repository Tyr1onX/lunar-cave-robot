#ifndef __RC_CONTROL_MODE_H
#define __RC_CONTROL_MODE_H

#include <stdint.h>

#define RC_CONTROL_PWM_DOWN_MAX  (1300u)
#define RC_CONTROL_PWM_UP_MIN    (1700u)

typedef enum
{
    RC_CONTROL_DISABLED = 0,
    RC_CONTROL_SERIAL,
    RC_CONTROL_SBUS
} rc_control_mode_e;

rc_control_mode_e rc_control_mode_from_pwm(uint16_t pwm);

#endif /* __RC_CONTROL_MODE_H */
