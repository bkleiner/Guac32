#pragma once

#include <stdbool.h>
#include <stdint.h>

void dshot_init();
bool dshot_read(uint16_t *val);