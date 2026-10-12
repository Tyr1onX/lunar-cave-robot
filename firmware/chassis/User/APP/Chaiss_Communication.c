#include "Chaiss_Communication.h"
#include "chassis_command.h"
#include "usart.h"
#include "string.h"
#include "math.h"

volatile Chaiss_Communication_t g_chaiss_communication;

static uint8_t uart1_rx_dma[CHAISS_RX_DMA_SIZE];
static uint8_t stream_buffer[CHAISS_FRAME_SIZE * 2u];
static uint16_t stream_length;

static float read_float_le(const uint8_t *bytes)
{
    uint32_t raw = ((uint32_t)bytes[0]) |
                   ((uint32_t)bytes[1] << 8) |
                   ((uint32_t)bytes[2] << 16) |
                   ((uint32_t)bytes[3] << 24);
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void remove_stream_prefix(uint16_t count)
{
    if (count >= stream_length)
    {
        stream_length = 0u;
        return;
    }

    memmove(stream_buffer, &stream_buffer[count], stream_length - count);
    stream_length = (uint16_t)(stream_length - count);
}

static void parse_stream(void)
{
    while (stream_length > 0u)
    {
        float vx;
        float vy;
        float vz;

        if (stream_buffer[0] != CHAISS_FRAME_HEAD)
        {
            remove_stream_prefix(1u);
            g_chaiss_communication.invalid_frame_count++;
            continue;
        }

        if (stream_length < CHAISS_FRAME_SIZE)
        {
            return;
        }

        if (stream_buffer[CHAISS_FRAME_SIZE - 1u] != CHAISS_FRAME_TAIL)
        {
            remove_stream_prefix(1u);
            g_chaiss_communication.invalid_frame_count++;
            continue;
        }

        vx = read_float_le(&stream_buffer[1]);
        vy = read_float_le(&stream_buffer[5]);
        vz = read_float_le(&stream_buffer[9]);

        if (isfinite(vx) && isfinite(vy) && isfinite(vz))
        {
            g_chaiss_communication.last_frame_ms = HAL_GetTick();
            g_chaiss_communication.valid_frame_count++;
            g_chaiss_communication.online = 1u;

            if (g_chaiss_communication.command_enabled != 0u)
            {
                g_chaiss_communication.vx = vx;
                g_chaiss_communication.vy = vy;
                g_chaiss_communication.vz = vz;
                g_chaiss_communication.accepted_command_count++;

                /*
                 * Special command: VX=0, VZ=0, VY=1 enters heading lock.
                 * Otherwise VY is ignored by the differential chassis.
                 */
                if ((fabsf(vx) <= 0.0001f) &&
                    (fabsf(vz) <= 0.0001f) &&
                    (fabsf(vy - 1.0f) <= 0.0001f))
                {
                    chassis_command_publish_heading_lock(CHASSIS_CMD_ARM);
                }
                else
                {
                    chassis_command_publish(vx, vz, 1u, CHASSIS_CMD_ARM);
                }
            }
            else
            {
                /* Valid frame received while CH8 is not in serial mode: discard it. */
                g_chaiss_communication.blocked_frame_count++;
            }
        }
        else
        {
            g_chaiss_communication.invalid_frame_count++;
        }

        remove_stream_prefix(CHAISS_FRAME_SIZE);
    }
}

static void append_rx_bytes(const uint8_t *data, uint16_t size)
{
    uint16_t index = 0u;

    while (index < size)
    {
        uint16_t free_size = (uint16_t)(sizeof(stream_buffer) - stream_length);
        uint16_t copy_size;

        if (free_size == 0u)
        {
            remove_stream_prefix(1u);
            free_size = 1u;
            g_chaiss_communication.invalid_frame_count++;
        }

        copy_size = (uint16_t)(size - index);
        if (copy_size > free_size)
        {
            copy_size = free_size;
        }

        memcpy(&stream_buffer[stream_length], &data[index], copy_size);
        stream_length = (uint16_t)(stream_length + copy_size);
        index = (uint16_t)(index + copy_size);
        parse_stream();
    }
}

void Chaiss_Communication_Init(void)
{
    memset((void *)&g_chaiss_communication, 0,
           sizeof(g_chaiss_communication));
    stream_length = 0u;
    memset(uart1_rx_dma, 0, sizeof(uart1_rx_dma));

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                       uart1_rx_dma,
                                       CHAISS_RX_DMA_SIZE);
    if (huart1.hdmarx != 0)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void Chaiss_Communication_RxEvent(uint16_t size)
{
    if (size > CHAISS_RX_DMA_SIZE)
    {
        size = CHAISS_RX_DMA_SIZE;
    }

    append_rx_bytes(uart1_rx_dma, size);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                       uart1_rx_dma,
                                       CHAISS_RX_DMA_SIZE);
    if (huart1.hdmarx != 0)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void Chaiss_Communication_UARTError(void)
{
    (void)HAL_UART_AbortReceive(&huart1);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                       uart1_rx_dma,
                                       CHAISS_RX_DMA_SIZE);
    if (huart1.hdmarx != 0)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void Chaiss_Communication_SetCommandEnabled(uint8_t enabled)
{
    g_chaiss_communication.command_enabled = (enabled != 0u) ? 1u : 0u;
}

uint8_t Chaiss_Communication_IsOnline(void)
{
    if ((g_chaiss_communication.online != 0u) &&
        ((HAL_GetTick() - g_chaiss_communication.last_frame_ms) <=
         CHAISS_ONLINE_TIMEOUT_MS))
    {
        return 1u;
    }

    g_chaiss_communication.online = 0u;
    return 0u;
}

uint8_t Chaiss_Communication_HasCommand(void)
{
    return (g_chaiss_communication.accepted_command_count > 0u) ? 1u : 0u;
}
