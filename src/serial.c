#include "serial.h"

#include <stdarg.h>
#include <stdio.h>

#include "gpio.h"
#include "mcu.h"
#include "targets.h"

void serial_init() {
  gpio_mode(TELE_PORT, TELE_PIN, GPIO_MODE_MUX, GPIO_PULL_NONE);
  gpio_pin_mux_config(TELE_PORT, TELE_PIN_SOURCE, TELE_PIN_MUX);

  crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
  usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART1, TRUE);
  usart_enable(USART1, TRUE);
}

void serial_write_byte(const uint8_t data) {
  while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
    ;
  usart_data_transmit(USART1, data);
}

void serial_write(const uint8_t *data, const uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    serial_write_byte(data[i]);
  }
}

void serial_printf(const char *fmt, ...) {
  va_list args1;
  va_start(args1, fmt);

  va_list args2;
  va_copy(args2, args1);

  const uint32_t len = vsnprintf(NULL, 0, fmt, args1);
  uint8_t buf[1 + len];
  va_end(args1);

  vsnprintf((char *)buf, sizeof buf, fmt, args2);
  va_end(args2);

  serial_write(buf, len);
}