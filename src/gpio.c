#include "gpio.h"

void gpio_mode(gpio_type *gpio_x, uint16_t pins, uint32_t mode, uint32_t pull_up_down) {
  gpio_init_type cfg;
  cfg.gpio_pins = pins;
  cfg.gpio_mode = mode;
  cfg.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  cfg.gpio_pull = pull_up_down;
  cfg.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(gpio_x, &cfg);
}

void gpio_pin_toggle(gpio_type *gpio_x, uint16_t pins) {
  if (gpio_output_data_bit_read(gpio_x, pins)) {
    gpio_bits_reset(gpio_x, pins);
  } else {
    gpio_bits_set(gpio_x, pins);
  }
}