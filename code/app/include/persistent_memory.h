#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Memory locations:
#define PERSIST_ID_STATE 1

bool persist_init(void);

bool persist_write(uint16_t id, const void *data, size_t len);

bool persist_read(uint16_t id, void *data, size_t len);
