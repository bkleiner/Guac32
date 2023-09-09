#pragma once

#include "mcu.h"

void gpio_mode(gpio_type *gpio_x, uint16_t pins, uint32_t mode, uint32_t pull_up_down);
void gpio_pin_toggle(gpio_type *gpio_x, uint16_t pins);