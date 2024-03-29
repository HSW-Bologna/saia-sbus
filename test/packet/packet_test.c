#include <stdint.h>
#include <stdlib.h>
#include "sbus/packet.h"
#include "unity.h"

void setUp() {}

void tearDown() {}

void try_simple_packet(uint8_t address, uint8_t command, uint8_t *data, size_t data_len) {
    uint16_t       buffer[256]  = {0};
    sbus_request_t request_sent = {
        .destination = address,
        .command     = command,
        .data_len    = data_len,
    };
    memcpy(request_sent.data, data, data_len);
    sbus_packet_serialize_request(buffer, &request_sent);

    sbus_request_t request;
    size_t         len = 2 + data_len + 2;
    int            res = sbus_packet_parse_request(buffer, &len, &request);

    TEST_ASSERT_EQUAL(SBUS_OK, res);
    TEST_ASSERT_EQUAL(command, request.command);
    TEST_ASSERT_EQUAL(address, request.destination);
    TEST_ASSERT_EQUAL(2 + data_len + 2, len);
}


void try_incomplete_packet(uint8_t address, uint8_t command, uint8_t *data, size_t data_len) {
    if (data_len > 0)
        data_len = data_len / 2 - 1;

    uint16_t buffer[256] = {SBUS_ADDRESS(address), command};

    for (size_t i = 0; i < data_len; i++)
        buffer[2 + i] = data[i];

    sbus_request_t request;
    size_t         len = 2 + data_len;
    int            res = sbus_packet_parse_request(buffer, &len, &request);

    TEST_ASSERT_EQUAL(SBUS_INCOMPLETE_PACKET, res);
    TEST_ASSERT_EQUAL(0, len);
}


void try_noise_packet(uint8_t address, uint8_t command, uint8_t *data, size_t data_len) {
    uint16_t buffer[256 + 16] = {0};

    buffer[16]     = SBUS_ADDRESS(address);
    buffer[16 + 1] = command;
    for (size_t i = 0; i < data_len; i++)
        buffer[16 + 2 + i] = data[i];
    uint16_t crc = sbus_crc16_9bit(buffer, 16 + 2 + data_len);

    buffer[16 + 2 + data_len]     = (crc >> 8) & 0xFF;
    buffer[16 + 2 + data_len + 1] = crc & 0xFF;

    sbus_request_t request;

    size_t len = 16 + 2 + 1;
    int    res = sbus_packet_parse_request(buffer, &len, &request);

    TEST_ASSERT_EQUAL(SBUS_INCOMPLETE_PACKET, res);
    TEST_ASSERT_EQUAL(16, len);

    len = 2 + data_len + 2;
    res = sbus_packet_parse_request(&buffer[16], &len, &request);
    TEST_ASSERT_EQUAL(SBUS_OK, res);
    TEST_ASSERT_EQUAL(command, request.command);
    TEST_ASSERT_EQUAL(address, request.destination);
    TEST_ASSERT_EQUAL(2 + data_len + 2, len);

    len = 2 + data_len + 2 + 10;
    res = sbus_packet_parse_request(&buffer[6], &len, &request);
    TEST_ASSERT_EQUAL(SBUS_OK, res);
    TEST_ASSERT_EQUAL(command, request.command);
    TEST_ASSERT_EQUAL(address, request.destination);
    TEST_ASSERT_EQUAL(2 + data_len + 2 + 10, len);
}

void try_packet(uint8_t address, uint8_t command, uint8_t *data, size_t data_len) {
    try_simple_packet(address, command, data, data_len);
    try_incomplete_packet(address, command, data, data_len);
    try_noise_packet(address, command, data, data_len);
}


void test_read_packet() {
    uint8_t buffer[256] = {0};
    try_packet(1, SBUS_COMMAND_READ_COUNTER, buffer, 3);
    try_packet(1, SBUS_COMMAND_READ_DISPLAY_REGISTER, NULL, 0);
    try_packet(1, SBUS_COMMAND_READ_FLAG, buffer, 3);
    try_packet(1, SBUS_COMMAND_READ_INPUT, buffer, 3);
    try_packet(1, SBUS_COMMAND_READ_REAL_TIME_CLOCK, NULL, 0);
    try_packet(1, SBUS_COMMAND_READ_OUTPUT, buffer, 3);
    try_packet(1, SBUS_COMMAND_READ_REGISTER, buffer, 3);
    try_packet(1, SBUS_COMMAND_READ_TIMER, buffer, 3);
    try_packet(1, SBUS_COMMAND_READ_PCD_STATUS_CPU_0, NULL, 0);
    try_packet(1, SBUS_COMMAND_READ_PCD_STATUS_CPU_1, NULL, 0);
    try_packet(1, SBUS_COMMAND_READ_PCD_STATUS_CPU_2, NULL, 0);
    try_packet(1, SBUS_COMMAND_READ_PCD_STATUS_CPU_3, NULL, 0);
    try_packet(1, SBUS_COMMAND_READ_PCD_STATUS_CPU_4, NULL, 0);
    try_packet(1, SBUS_COMMAND_READ_PCD_STATUS_CPU_5, NULL, 0);
    try_packet(1, SBUS_COMMAND_READ_PCD_STATUS_CPU_6, NULL, 0);
    try_packet(1, SBUS_COMMAND_READ_PCD_STATUS_SELF, NULL, 0);
    try_packet(1, SBUS_COMMAND_READ_STATION_NUMBER, NULL, 0);
}

void test_unknown_command() {
    uint16_t buffer[4] = {SBUS_ADDRESS(1), 0xFF};
    uint16_t crc       = sbus_crc16_9bit(buffer, 2);

    buffer[2]     = (crc >> 8) & 0xFF;
    buffer[2 + 1] = crc & 0xFF;

    sbus_request_t request;
    size_t         len = 4;
    int            res = sbus_packet_parse_request(buffer, &len, &request);

    TEST_ASSERT_EQUAL(SBUS_UNKNOWN_COMMAND, res);
}


void test_write_counter() {
    uint8_t buffer[256] = {0};

    buffer[0] = 5;
    try_packet(1, SBUS_COMMAND_WRITE_COUNTER, buffer, buffer[0] + 2);

    buffer[0] = 33;
    try_packet(1, SBUS_COMMAND_WRITE_COUNTER, buffer, buffer[0] + 2);
}

void test_write_flag() {
    uint8_t buffer[256] = {0};

    buffer[0] = 2;
    try_packet(1, SBUS_COMMAND_WRITE_FLAG, buffer, buffer[0] + 2);

    buffer[0] = 17;
    try_packet(1, SBUS_COMMAND_WRITE_FLAG, buffer, buffer[0] + 2);
}

void test_write_output() {
    uint8_t buffer[256] = {0};

    buffer[0] = 2;
    try_packet(1, SBUS_COMMAND_WRITE_FLAG, buffer, buffer[0] + 2);

    buffer[0] = 17;
    try_packet(1, SBUS_COMMAND_WRITE_FLAG, buffer, buffer[0] + 2);
}

void test_write_real_time_clock() {
    uint8_t buffer[256] = {0};
    try_packet(1, SBUS_COMMAND_WRITE_REAL_TIME_CLOCK, buffer, 6);
}

void test_write_register() {
    uint8_t buffer[256] = {0};

    buffer[0] = 5;
    try_packet(1, SBUS_COMMAND_WRITE_REGISTER, buffer, buffer[0] + 2);

    buffer[0] = 33;
    try_packet(1, SBUS_COMMAND_WRITE_REGISTER, buffer, buffer[0] + 2);
}

void test_write_timer() {
    uint8_t buffer[256] = {0};

    buffer[0] = 5;
    try_packet(1, SBUS_COMMAND_WRITE_TIMER, buffer, buffer[0] + 2);

    buffer[0] = 33;
    try_packet(1, SBUS_COMMAND_WRITE_TIMER, buffer, buffer[0] + 2);
}

void test_read_registers() {
    uint8_t  rcount      = 2;
    size_t   data_len    = (rcount + 1) * 4;
    uint16_t buffer[256] = {0};
    uint16_t crc         = sbus_crc16_9bit(buffer, data_len);
    buffer[data_len]     = (crc >> 8) & 0xFF;
    buffer[data_len + 1] = crc & 0xFF;

    uint8_t  buffer_8bit[256] = {0};
    uint16_t crc_8bit         = sbus_crc16_8bit(buffer_8bit, data_len);
    buffer_8bit[data_len]     = (crc_8bit >> 8) & 0xFF;
    buffer_8bit[data_len + 1] = crc_8bit & 0xFF;
    TEST_ASSERT_EQUAL(crc_8bit, crc);

    sbus_request_t request = SBUS_REQUEST(1, SBUS_COMMAND_READ_REGISTER, {rcount, 0, 0});

    size_t len = 256;
    int    res = sbus_packet_validate_response_9bit(&request, buffer, &len);
    TEST_ASSERT_EQUAL(SBUS_OK, res);
    TEST_ASSERT_EQUAL(data_len + 2, len);

    len = 256;
    res = sbus_packet_validate_response_8bit(&request, buffer_8bit, &len);
    TEST_ASSERT_EQUAL(SBUS_OK, res);
    TEST_ASSERT_EQUAL(data_len + 2, len);
}


void test_field_data() {
    uint16_t       data[]  = {0x00, 0x02, 0x00, 0x4B, 0x02, 0x08, 0x00, 0xCA, 0x00, 0xB4,
                       0x00, 0x00, 0xF1, 0x22, 0x41, 0x01, 0x64, 0x50, 0xCA, 0x00};
    sbus_request_t request = SBUS_REQUEST(75, 6, {3, 0, 9});

    size_t             len = 20;
    sbus_result_t res = sbus_packet_validate_response_9bit(&request, data, &len);

    TEST_ASSERT_EQUAL(res, SBUS_OK);
}