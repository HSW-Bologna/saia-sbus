#include <stdlib.h>
#include <assert.h>

#include "packet.h"


#define ADDRESS_MASK  0x0100
#define IS_ADDRESS(x) (((x)&ADDRESS_MASK) > 0)
#define IS_DATA(x)    (!IS_ADDRESS(x))

static int unpack_command(sbus_command_code_t command, uint16_t *buffer, size_t *len, sbus_request_t *request);
static int check_data(uint16_t *buffer, size_t len);


size_t sbus_packet_serialize_request(uint16_t *buffer, const sbus_request_t *request) {
    buffer[0] = SBUS_ADDRESS(request->destination);
    buffer[1] = (uint16_t)request->command;
    for (size_t i = 0; i < request->data_len; i++) {
        buffer[2 + i] = request->data[i];
    }
    uint16_t crc                      = sbus_crc16_9bit(buffer, 2 + request->data_len);
    buffer[2 + request->data_len]     = (crc >> 8) & 0xFF;
    buffer[2 + request->data_len + 1] = crc & 0xFF;
    return 4 + request->data_len;
}


sbus_result_t sbus_packet_parse_request(uint16_t *buffer, size_t *len, sbus_request_t *request) {
    for (size_t i = 0; i < *len; i++) {
        if (IS_ADDRESS(buffer[i])) {
            size_t  start       = i;
            uint8_t destination = (uint8_t)(buffer[start] & 0xFF);

            if (start + 3 > *len) {
                *len = start;
                return SBUS_INCOMPLETE_PACKET;     // The packet is not complete yet
            }

            if (IS_ADDRESS(buffer[start + 1])) {
                *len = start + 1;
                return SBUS_INVALID_DATA;     // Invalid data
            }

            uint8_t command  = (uint8_t)(buffer[start + 1] & 0xFF);
            size_t  data_len = *len - (start + 2);
            int     res      = unpack_command(command, &buffer[start + 2], &data_len, request);

            switch (res) {
                case SBUS_OK:
                    break;

                case SBUS_INCOMPLETE_PACKET:
                    *len = start;
                    return res;

                case SBUS_UNKNOWN_COMMAND:
                case SBUS_INVALID_DATA:
                default:
                    return res;     // Do not update len; everything is to be thrown away
            }

            if (start + 2 + data_len + 2 > *len) {
                *len = start;
                return SBUS_INCOMPLETE_PACKET;     // The packet is not complete yet
            }

            *len                 = start + 2 + data_len + 2;
            uint16_t crc         = sbus_crc16_9bit(&buffer[start], *len - 2 - start);
            uint16_t found_crc   = (uint16_t)((buffer[start + 2 + data_len] << 8) | buffer[start + 2 + data_len + 1]);
            request->destination = destination;

            if (crc != found_crc)
                return SBUS_WRONG_CRC;
            else
                return SBUS_OK;
        }
    }

    return SBUS_NOT_FOUND;
}


sbus_result_t sbus_packet_validate_response_9bit(sbus_request_t *request, uint16_t *buffer, size_t *len) {
    size_t required_len = sbus_packet_response_length(request);

    if (required_len == 0) {
        *len = 0;
        return SBUS_OK;
    }

    if (*len < required_len) {
        *len = 0;
        return SBUS_INCOMPLETE_PACKET;
    }

    int index = check_data(buffer, required_len);
    if (index >= 0) {
        *len = (size_t)index;
        return SBUS_NOT_FOUND;
    }

    switch (request->command) {
        case SBUS_COMMAND_WRITE_COUNTER:
        case SBUS_COMMAND_WRITE_FLAG:
        case SBUS_COMMAND_WRITE_REAL_TIME_CLOCK:
        case SBUS_COMMAND_WRITE_OUTPUT:
        case SBUS_COMMAND_WRITE_REGISTER:
        case SBUS_COMMAND_WRITE_TIMER:
            if ((buffer[0] == SBUS_ACK || buffer[0] == SBUS_NAK) && buffer[1] == 0x00)
                return SBUS_OK;
            else
                return SBUS_INVALID_DATA;

        default: {
            uint16_t crc       = sbus_crc16_9bit(buffer, required_len - 2);
            uint16_t found_crc = (uint16_t)((buffer[required_len - 2] << 8) | buffer[required_len - 1]);
            *len               = required_len;

            if (crc != found_crc)
                return SBUS_WRONG_CRC;
            else
                return SBUS_OK;
        }
    }

    return SBUS_OK;
}


sbus_result_t sbus_packet_validate_response_8bit(sbus_request_t *request, uint8_t *buffer, size_t *len) {
    size_t required_len = sbus_packet_response_length(request);

    if (required_len == 0) {
        *len = 0;
        return SBUS_OK;
    }

    if (*len < required_len) {
        *len = 0;
        return SBUS_INCOMPLETE_PACKET;
    }

    switch (request->command) {
        case SBUS_COMMAND_WRITE_COUNTER:
        case SBUS_COMMAND_WRITE_FLAG:
        case SBUS_COMMAND_WRITE_REAL_TIME_CLOCK:
        case SBUS_COMMAND_WRITE_OUTPUT:
        case SBUS_COMMAND_WRITE_REGISTER:
        case SBUS_COMMAND_WRITE_TIMER:
            if ((buffer[0] == SBUS_ACK || buffer[0] == SBUS_NAK) && buffer[1] == 0x00) {
                return SBUS_OK;
            } else {
                return SBUS_INVALID_DATA;
            }

        default: {
            uint16_t crc       = sbus_crc16_8bit(buffer, required_len - 2);
            uint16_t found_crc = (uint16_t)((buffer[required_len - 2] << 8) | buffer[required_len - 1]);
            *len               = required_len;

            if (crc != found_crc)
                return SBUS_WRONG_CRC;
            else
                return SBUS_OK;
        }
    }

    return SBUS_OK;
}



size_t sbus_packet_response_length(sbus_request_t *request) {
    if (request->destination == SBUS_BROADCAST_ADDRESS)     // No answer to broadcast messages
        return 0;

    switch (request->command) {
        case SBUS_COMMAND_READ_COUNTER:
        case SBUS_COMMAND_READ_REGISTER:
        case SBUS_COMMAND_READ_TIMER:
            return (SBUS_PACKET_R_COUNT(request) + 1) * 4 + 2;

        case SBUS_COMMAND_READ_DISPLAY_REGISTER:
            return 4 + 2;

        case SBUS_COMMAND_READ_FLAG:
        case SBUS_COMMAND_READ_INPUT:
        case SBUS_COMMAND_READ_OUTPUT:
            return (SBUS_PACKET_R_COUNT(request) + 1) / 8 + 2;

        case SBUS_COMMAND_READ_REAL_TIME_CLOCK:
            return 6 + 2;

        case SBUS_COMMAND_WRITE_COUNTER:
        case SBUS_COMMAND_WRITE_FLAG:
        case SBUS_COMMAND_WRITE_REAL_TIME_CLOCK:
        case SBUS_COMMAND_WRITE_OUTPUT:
        case SBUS_COMMAND_WRITE_REGISTER:
        case SBUS_COMMAND_WRITE_TIMER:
            return 2;

        case SBUS_COMMAND_READ_PCD_STATUS_CPU_0:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_1:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_2:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_3:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_4:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_5:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_6:
        case SBUS_COMMAND_READ_PCD_STATUS_SELF:
        case SBUS_COMMAND_READ_STATION_NUMBER:
            return 1 + 2;

        default:
            assert(0);
            break;
    }

    return 0;
}


int sbus_packet_serialize_register_read_response(uint16_t *buffer, size_t len, uint32_t *registers, size_t count,
                                                 sbus_request_t *request) {
    size_t expected_len = sbus_packet_response_length(request);
    if (len < expected_len || request->command != SBUS_COMMAND_READ_REGISTER) {
        return -1;
    }

    for (size_t i = 0; i < count; i++) {
        buffer[i * 4 + 0] = (registers[i] >> 24) & 0xFF;
        buffer[i * 4 + 1] = (registers[i] >> 16) & 0xFF;
        buffer[i * 4 + 2] = (registers[i] >> 8) & 0xFF;
        buffer[i * 4 + 3] = registers[i] & 0xFF;
    }

    uint16_t crc          = sbus_crc16_9bit(buffer, count * 4);
    buffer[count * 4]     = (crc >> 8) & 0xFF;
    buffer[count * 4 + 1] = crc & 0xFF;

    return (int)count * 4 + 2;
}


uint16_t sbus_crc16_8bit(uint8_t *buffer, size_t length) {
    int      i;
    uint16_t crc;
    uint16_t data;
    if (length == 0)
        return (0);
    crc = 0;
    do {
        data = *buffer;
        crc  = (uint16_t)(crc ^ (data << 8));
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc <<= 1;
        }
        buffer++;
    } while (--length);
    return crc;
}


uint16_t sbus_crc16_9bit(uint16_t *buffer, size_t length) {
    int      i;
    uint16_t crc;
    uint16_t data;
    if (length == 0)
        return (0);
    crc = 0;
    do {
        data = (uint8_t)((*buffer) & 0xFF);
        crc  = (uint16_t)(crc ^ (data << 8));
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc <<= 1;
        }
        buffer++;
    } while (--length);
    return crc;
}


static int unpack_command(sbus_command_code_t command, uint16_t *buffer, size_t *len, sbus_request_t *request) {
#define CHECK_LEN(required)                                                                                            \
    if (*len < required)                                                                                               \
        return SBUS_INCOMPLETE_PACKET;                                                                                 \
    *len = required;

    request->command = command;

    switch (command) {
        case SBUS_COMMAND_READ_DISPLAY_REGISTER:
        case SBUS_COMMAND_READ_REAL_TIME_CLOCK:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_0:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_1:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_2:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_3:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_4:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_5:
        case SBUS_COMMAND_READ_PCD_STATUS_CPU_6:
        case SBUS_COMMAND_READ_PCD_STATUS_SELF:
        case SBUS_COMMAND_READ_STATION_NUMBER:
            *len = 0;
            break;

        case SBUS_COMMAND_READ_COUNTER:
        case SBUS_COMMAND_READ_FLAG:
        case SBUS_COMMAND_READ_INPUT:
        case SBUS_COMMAND_READ_OUTPUT:
        case SBUS_COMMAND_READ_REGISTER:
        case SBUS_COMMAND_READ_TIMER:
            CHECK_LEN(3);
            break;

        case SBUS_COMMAND_WRITE_COUNTER:
        case SBUS_COMMAND_WRITE_REGISTER:
        case SBUS_COMMAND_WRITE_TIMER: {
            if (*len < 1)
                return SBUS_INCOMPLETE_PACKET;

            if (buffer[0] < 5 || buffer[0] > 129)
                return SBUS_INVALID_DATA;
            if ((buffer[0] - 1) % 4 != 0)
                return SBUS_INVALID_DATA;

            size_t total_len = 2 + buffer[0];
            CHECK_LEN(total_len);
            break;
        }

        case SBUS_COMMAND_WRITE_OUTPUT:
        case SBUS_COMMAND_WRITE_FLAG: {
            if (*len < 3)
                return SBUS_INCOMPLETE_PACKET;

            if (buffer[0] < 2 || buffer[0] > 17)
                return SBUS_INVALID_DATA;
            if (buffer[2] > 127)
                return SBUS_INVALID_DATA;

            size_t total_len = 2 + buffer[0];
            CHECK_LEN(total_len);
            break;
        }

        case SBUS_COMMAND_WRITE_REAL_TIME_CLOCK:
            CHECK_LEN(6);
            break;

        default:
            return SBUS_UNKNOWN_COMMAND;
    }

    if (check_data(buffer, *len) >= 0) {
        return SBUS_INVALID_DATA;
    }

    request->data_len = (uint8_t)*len;
    for (size_t i = 0; i < *len; i++) {
        request->data[i] = (uint8_t)buffer[i];
    }

    return SBUS_OK;
}


static int check_data(uint16_t *buffer, size_t len) {
    for (int i = 0; i < (int)len; i++) {
        if (IS_ADDRESS(buffer[i]))
            return i;
    }

    return -1;
}
