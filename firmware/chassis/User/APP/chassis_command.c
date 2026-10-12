#include "chassis_command.h"
#include "main.h"

/*
 * Sequence lock: the writer makes the sequence odd while updating and even
 * when finished. This gives the 200 Hz chassis task a coherent snapshot
 * without coupling the command interface to a particular RTOS queue.
 */
static volatile uint32_t command_sequence;
static volatile chassis_command_t shared_command;
/* Central safety gate controlled by the CH8 three-position selector. */
static volatile uint8_t safety_enabled;

static void chassis_command_write(float linear_x, float angular_z,
                                  uint8_t enabled,
                                  chassis_cmd_source_e source,
                                  chassis_control_mode_e mode)
{
    command_sequence++;
    shared_command.linear_x = linear_x;
    shared_command.angular_z = angular_z;
    shared_command.update_ms = HAL_GetTick();
    shared_command.source = source;
    shared_command.mode = mode;
    shared_command.enabled = enabled;
    command_sequence++;
}

void chassis_command_publish(float linear_x, float angular_z,
                             uint8_t enabled, chassis_cmd_source_e source)
{
    if (safety_enabled == 0u)
    {
        return;
    }

    chassis_command_write(linear_x, angular_z, enabled, source,
                          CHASSIS_MODE_VELOCITY);
}

void chassis_command_publish_heading_lock(chassis_cmd_source_e source)
{
    if (safety_enabled == 0u)
    {
        return;
    }

    chassis_command_write(0.0f, 0.0f, 1u, source,
                          CHASSIS_MODE_HEADING_LOCK);
}

void chassis_command_set_safety_enabled(uint8_t enabled)
{
    if (enabled != 0u)
    {
        safety_enabled = 1u;
        return;
    }

    /* Close the gate before writing stop, so an IRQ cannot overwrite it. */
    safety_enabled = 0u;
    chassis_command_write(0.0f, 0.0f, 0u, CHASSIS_CMD_RC,
                          CHASSIS_MODE_VELOCITY);
}

uint8_t chassis_command_is_safety_enabled(void)
{
    return safety_enabled;
}

void chassis_command_stop(chassis_cmd_source_e source)
{
    /* A stop command is always allowed, including while the gate is closed. */
    chassis_command_write(0.0f, 0.0f, 0u, source,
                          CHASSIS_MODE_VELOCITY);
}

uint8_t chassis_command_read(chassis_command_t *command)
{
    uint32_t before;
    uint32_t after;
    uint8_t retry;

    if (command == 0)
    {
        return 0u;
    }

    for (retry = 0u; retry < 3u; retry++)
    {
        before = command_sequence;
        if ((before & 1u) != 0u)
        {
            continue;
        }

        command->linear_x = shared_command.linear_x;
        command->angular_z = shared_command.angular_z;
        command->update_ms = shared_command.update_ms;
        command->source = shared_command.source;
        command->mode = shared_command.mode;
        command->enabled = shared_command.enabled;

        after = command_sequence;
        if ((before == after) && ((after & 1u) == 0u))
        {
            return 1u;
        }
    }

    return 0u;
}

uint8_t chassis_command_is_fresh(const chassis_command_t *command,
                                 uint32_t now_ms, uint32_t max_age_ms)
{
    if (command == 0)
    {
        return 0u;
    }

    /* Unsigned subtraction is intentional so the millisecond tick may wrap. */
    return ((uint32_t)(now_ms - command->update_ms) <= max_age_ms) ? 1u : 0u;
}
