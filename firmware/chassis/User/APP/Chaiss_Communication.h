#ifndef __CHAISS_COMMUNICATION_H
#define __CHAISS_COMMUNICATION_H

#include "main.h"

/*
 * Upper-computer chassis command frame (14 bytes):
 *   byte 0      : 0xA5
 *   bytes 1..4  : VX, float32, little-endian, m/s
 *   bytes 5..8  : VY, float32, little-endian; VY=1 with VX=VZ=0 enables
 *                 heading lock, otherwise ignored by differential drive
 *   bytes 9..12 : VZ, float32, little-endian, rad/s
 *   byte 13     : 0x4A
 */
#define CHAISS_FRAME_HEAD       (0xA5u)
#define CHAISS_FRAME_TAIL       (0x4Au)
#define CHAISS_FRAME_SIZE       (14u)
#define CHAISS_RX_DMA_SIZE      (64u)
#define CHAISS_ONLINE_TIMEOUT_MS (200u)

typedef struct
{
    float vx;
    float vy;
    float vz;
    uint32_t last_frame_ms;
    uint32_t valid_frame_count;
    uint32_t accepted_command_count;
    uint32_t blocked_frame_count;
    uint32_t invalid_frame_count;
    uint8_t online;
    uint8_t command_enabled;
} Chaiss_Communication_t;

extern volatile Chaiss_Communication_t g_chaiss_communication;

void Chaiss_Communication_Init(void);
void Chaiss_Communication_RxEvent(uint16_t size);
void Chaiss_Communication_UARTError(void);
void Chaiss_Communication_SetCommandEnabled(uint8_t enabled);
uint8_t Chaiss_Communication_IsOnline(void);
/* Returns 1 after the first accepted serial command, until the MCU is reset. */
uint8_t Chaiss_Communication_HasCommand(void);

#endif /* __CHAISS_COMMUNICATION_H */
