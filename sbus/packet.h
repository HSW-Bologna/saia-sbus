#ifndef SBUS_PACKET_H_INCLUDED
#define SBUS_PACKET_H_INCLUDED

#include <stdint.h>
#include <stdlib.h>

#define SBUS_PACKET(dest, command, data)                                                                               \
    ({                                                                                                                 \
        uint8_t        buffer[] = data;                                                                                \
        sbus_request_t request;                                                                                        \
        request.destination = dest;                                                                                    \
        request.command     = command;                                                                                 \
        request.data_len    = sizeof(buffer) / sizeof(uint8_t);                                                        \
        memcpy(request.data, buffer, request.data_len);                                                                \
        request;                                                                                                       \
    })

#define SBUS_PACKET_R_COUNT(packet)  (packet->data[0])
#define SBUS_PACKET_REG_ADDR(packet) ((uint16_t)(packet->data[1] << 8) && (uint16_t)(packet->data[2]))

#define SBUS_ADDRESS(addr) (0x0100 | addr)

typedef enum {
    SBUS_COMMAND_READ_COUNTER          = 0,
    SBUS_COMMAND_READ_DISPLAY_REGISTER = 1,
    SBUS_COMMAND_READ_FLAG             = 2,
    SBUS_COMMAND_READ_INPUT            = 3,
    SBUS_COMMAND_READ_REAL_TIME_CLOCK  = 4,
    SBUS_COMMAND_READ_OUTPUT           = 5,
    SBUS_COMMAND_READ_REGISTER         = 6,
    SBUS_COMMAND_READ_TIMER            = 7,
    SBUS_COMMAND_WRITE_COUNTER         = 10,
    SBUS_COMMAND_WRITE_FLAG            = 11,
    SBUS_COMMAND_WRITE_REAL_TIME_CLOCK = 12,
    SBUS_COMMAND_WRITE_OUTPUT          = 13,
    SBUS_COMMAND_WRITE_REGISTER        = 14,
    SBUS_COMMAND_WRITE_TIMER           = 15,
    SBUS_COMMAND_READ_PCD_STATUS_CPU_0 = 20,
    SBUS_COMMAND_READ_PCD_STATUS_CPU_1 = 21,
    SBUS_COMMAND_READ_PCD_STATUS_CPU_2 = 22,
    SBUS_COMMAND_READ_PCD_STATUS_CPU_3 = 23,
    SBUS_COMMAND_READ_PCD_STATUS_CPU_4 = 24,
    SBUS_COMMAND_READ_PCD_STATUS_CPU_5 = 25,
    SBUS_COMMAND_READ_PCD_STATUS_CPU_6 = 26,
    SBUS_COMMAND_READ_PCD_STATUS_SELF  = 27,
    SBUS_COMMAND_READ_STATION_NUMBER   = 29,
} sbus_command_code_t;


typedef enum {
    SBUS_OK                = 0,
    SBUS_INCOMPLETE_PACKET = -1,
    SBUS_INVALID_DATA      = -2,
    SBUS_UNKNOWN_COMMAND   = -3,
    SBUS_NOT_FOUND         = -4,
    SBUS_WRONG_CRC         = -5,
} sbus_result_code_t;


typedef struct {
    uint8_t             destination;
    sbus_command_code_t command;
    uint8_t             data_len;
    uint8_t             data[256];
} sbus_request_t;

size_t sbus_crc16_9bit(uint16_t *buffer, size_t length);
size_t sbus_crc16_8bit(uint8_t *buffer, size_t length);
int    sbus_packet_parse_request(uint16_t *buffer, size_t *len, sbus_request_t *request);

#endif