#include "Chaiss_Communication.h"
#include "chassis_command.h"
#include "main.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;
DMA_HandleTypeDef test_dma;

static uint32_t now_ms;
static uint8_t *rx_dma_buffer;
static uint16_t rx_dma_size;

uint32_t HAL_GetTick(void) { return now_ms; }

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                                uint8_t *buffer,
                                                uint16_t size)
{
    assert(uart == &huart1);
    rx_dma_buffer = buffer;
    rx_dma_size = size;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    assert(uart == &huart1);
    return HAL_OK;
}

static void put_float_le(uint8_t *out, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    out[0] = (uint8_t)raw;
    out[1] = (uint8_t)(raw >> 8);
    out[2] = (uint8_t)(raw >> 16);
    out[3] = (uint8_t)(raw >> 24);
}

static void make_frame(uint8_t out[CHAISS_FRAME_SIZE], float vx, float vy, float vz)
{
    out[0] = CHAISS_FRAME_HEAD;
    put_float_le(&out[1], vx);
    put_float_le(&out[5], vy);
    put_float_le(&out[9], vz);
    out[13] = CHAISS_FRAME_TAIL;
}

static void inject(const uint8_t *bytes, uint16_t size)
{
    assert(rx_dma_buffer != NULL);
    assert(size <= rx_dma_size);
    memcpy(rx_dma_buffer, bytes, size);
    Chaiss_Communication_RxEvent(size);
}

static chassis_command_t command_snapshot(void)
{
    chassis_command_t command;
    assert(chassis_command_read(&command) == 1u);
    return command;
}

int main(void)
{
    uint8_t frame[CHAISS_FRAME_SIZE];
    uint8_t pair[CHAISS_FRAME_SIZE * 2u + 2u];
    chassis_command_t command;
    chassis_command_t lease_probe = {0};

    Chaiss_Communication_Init();
    assert(rx_dma_buffer != NULL);
    assert(rx_dma_size == CHAISS_RX_DMA_SIZE);
    assert(chassis_command_is_safety_enabled() == 0u);

    /* The production lease predicate expires at 201 ms and tolerates tick wrap. */
    lease_probe.update_ms = 1000u;
    assert(chassis_command_is_fresh(&lease_probe, 1200u, 200u) == 1u);
    assert(chassis_command_is_fresh(&lease_probe, 1201u, 200u) == 0u);
    lease_probe.update_ms = UINT32_MAX - 4u;
    assert(chassis_command_is_fresh(&lease_probe, 5u, 10u) == 1u);

    /* A syntactically valid frame is observed but blocked until serial mode opens. */
    make_frame(frame, 0.2f, 0.0f, -0.4f);
    now_ms = 10u;
    inject(frame, sizeof(frame));
    assert(g_chaiss_communication.valid_frame_count == 1u);
    assert(g_chaiss_communication.blocked_frame_count == 1u);
    command = command_snapshot();
    assert(command.enabled == 0u);

    /* A split frame is retained and decoded after the remainder arrives. */
    chassis_command_set_safety_enabled(1u);
    Chaiss_Communication_SetCommandEnabled(1u);
    now_ms = 20u;
    make_frame(frame, 0.35f, 0.0f, -0.6f);
    inject(frame, 6u);
    assert(g_chaiss_communication.accepted_command_count == 0u);
    now_ms = 21u;
    inject(&frame[6], (uint16_t)(sizeof(frame) - 6u));
    assert(g_chaiss_communication.accepted_command_count == 1u);
    command = command_snapshot();
    assert(command.enabled == 1u);
    assert(command.source == CHASSIS_CMD_ARM);
    assert(command.mode == CHASSIS_MODE_VELOCITY);
    assert(fabsf(command.linear_x - 0.35f) < 1.0e-6f);
    assert(fabsf(command.angular_z + 0.6f) < 1.0e-6f);
    assert(command.update_ms == 21u);

    /* Back-to-back frames survive a single DMA idle event. */
    make_frame(&pair[0], 0.1f, 0.0f, 0.2f);
    make_frame(&pair[CHAISS_FRAME_SIZE], -0.1f, 0.0f, -0.2f);
    now_ms = 30u;
    inject(pair, sizeof(pair));
    assert(g_chaiss_communication.accepted_command_count == 3u);
    command = command_snapshot();
    assert(fabsf(command.linear_x + 0.1f) < 1.0e-6f);
    assert(fabsf(command.angular_z + 0.2f) < 1.0e-6f);

    /* Garbage and a bad tail resynchronize at the next frame header. */
    pair[0] = 0x12u;
    pair[1] = 0x34u;
    make_frame(&pair[2], 0.25f, 0.0f, 0.1f);
    pair[2u + 13u] = 0x00u;
    make_frame(&pair[CHAISS_FRAME_SIZE + 2u], 0.3f, 0.0f, -0.1f);
    now_ms = 40u;
    inject(pair, (uint16_t)(CHAISS_FRAME_SIZE * 2u + 2u));
    assert(g_chaiss_communication.invalid_frame_count > 0u);
    assert(g_chaiss_communication.accepted_command_count == 4u);
    command = command_snapshot();
    assert(fabsf(command.linear_x - 0.3f) < 1.0e-6f);

    /* Fault injection: a one-bit finite payload error passes because the wire format has no CRC. */
    make_frame(frame, 0.25f, 0.0f, 0.0f);
    frame[1] ^= 0x01u;
    now_ms = 45u;
    const uint32_t accepted_before_corruption = g_chaiss_communication.accepted_command_count;
    inject(frame, sizeof(frame));
    assert(g_chaiss_communication.accepted_command_count == accepted_before_corruption + 1u);
    command = command_snapshot();
    assert(command.linear_x != 0.25f);

    /* Fault injection: a UART error re-arms DMA but the production parser retains a partial frame. */
    make_frame(frame, 0.2f, 0.0f, -0.3f);
    now_ms = 47u;
    const uint32_t accepted_before_uart_error = g_chaiss_communication.accepted_command_count;
    inject(frame, 7u);
    Chaiss_Communication_UARTError();
    inject(&frame[7], (uint16_t)(sizeof(frame) - 7u));
    assert(g_chaiss_communication.accepted_command_count == accepted_before_uart_error + 1u);
    command = command_snapshot();
    assert(fabsf(command.linear_x - 0.2f) < 1.0e-6f);

    /* Non-finite payloads are rejected. */
    make_frame(frame, NAN, 0.0f, 0.0f);
    now_ms = 50u;
    const uint32_t accepted_before_nan = g_chaiss_communication.accepted_command_count;
    inject(frame, sizeof(frame));
    assert(g_chaiss_communication.accepted_command_count == accepted_before_nan);

    /* The distinctive legacy VY sentinel requests heading lock rather than velocity mode. */
    make_frame(frame, 0.0f, 1.0f, 0.0f);
    now_ms = 60u;
    inject(frame, sizeof(frame));
    command = command_snapshot();
    assert(command.mode == CHASSIS_MODE_HEADING_LOCK);
    assert(command.enabled == 1u);

    /* The firmware online telemetry expires after 200 ms; it is separate from the controller's 200 ms command lease. */
    now_ms += CHAISS_ONLINE_TIMEOUT_MS + 1u;
    assert(Chaiss_Communication_IsOnline() == 0u);
    assert(Chaiss_Communication_HasCommand() == 1u);

    /* A valid frame cannot bypass the independent chassis safety gate. */
    chassis_command_set_safety_enabled(0u);
    Chaiss_Communication_SetCommandEnabled(1u);
    make_frame(frame, 0.5f, 0.0f, 0.0f);
    now_ms += 1u;
    inject(frame, sizeof(frame));
    command = command_snapshot();
    assert(command.enabled == 0u);
    assert(chassis_command_is_safety_enabled() == 0u);

    puts("PASS chassis firmware parser host tests (production parser/command code, mocked HAL)");
    return 0;
}
