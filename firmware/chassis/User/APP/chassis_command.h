#ifndef __CHASSIS_COMMAND_H
#define __CHASSIS_COMMAND_H

#include "stdint.h"

/*
 * Unified velocity command used by the chassis controller.
 * linear_x:  forward velocity in m/s
 * angular_z: yaw rate in rad/s, positive counter-clockwise
 *
 * The RC receiver publishes this structure during development. The future
 * manipulator communication parser must call chassis_command_publish() with
 * its x and z values, so the controller and kinematics do not need changing.
 */
typedef enum
{
    CHASSIS_CMD_NONE = 0,
    CHASSIS_CMD_RC,
    CHASSIS_CMD_ARM,
    CHASSIS_CMD_TEST
} chassis_cmd_source_e;

typedef enum
{
    CHASSIS_MODE_VELOCITY = 0,
    CHASSIS_MODE_HEADING_LOCK
} chassis_control_mode_e;

typedef struct
{
    float linear_x;
    float angular_z;
    uint32_t update_ms;
    chassis_cmd_source_e source;
    chassis_control_mode_e mode;
    uint8_t enabled;
} chassis_command_t;

void chassis_command_publish(float linear_x, float angular_z,
                             uint8_t enabled, chassis_cmd_source_e source);
void chassis_command_set_safety_enabled(uint8_t enabled);
uint8_t chassis_command_is_safety_enabled(void);
void chassis_command_stop(chassis_cmd_source_e source);
void chassis_command_publish_heading_lock(chassis_cmd_source_e source);
uint8_t chassis_command_read(chassis_command_t *command);
uint8_t chassis_command_is_fresh(const chassis_command_t *command,
                                 uint32_t now_ms, uint32_t max_age_ms);

#endif /* __CHASSIS_COMMAND_H */
