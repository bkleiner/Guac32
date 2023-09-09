#pragma once

#include <stdbool.h>
#include <stdint.h>

void soft_serial_init();
bool soft_serial_read_byte(uint8_t *data);
void soft_serial_write_byte(uint8_t val);
void soft_serial_write(const uint8_t *data, const uint32_t length);