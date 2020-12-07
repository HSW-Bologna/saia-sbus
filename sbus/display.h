#ifndef SBUS_DISPLAY_H_INCLUDED
#define SBUS_DISPLAY_H_INCLUDED

#include <stdlib.h>

#include "packet.h"

size_t sbus_request_display(char *string, size_t len, sbus_request_t *request);

#endif