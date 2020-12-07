#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "packet.h"


size_t sbus_request_display(char *string, size_t len, sbus_request_t *request) {
    size_t final_length = 0;
    final_length +=
        snprintf(string + final_length, len, "Request for %i, command %i\n", request->destination, request->command);

    if (request->data_len > 0) {
        final_length += snprintf(string + final_length, len, "\tData:");
        for (size_t i = 0; i < request->data_len; i++)
            final_length += snprintf(string + final_length, len, " 0x%02X", request->data[i]);
    }

    final_length += snprintf(string + final_length, len, "\n");

    return final_length;
}