#pragma once

#include <stdint.h>

void serial_init();
void serial_write_byte(const uint8_t data);
void serial_write(const uint8_t *data, const uint32_t len);
void serial_printf(const char *fmt, ...);