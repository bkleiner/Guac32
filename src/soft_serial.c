#include "soft_serial.h"

#include "gpio.h"
#include "mcu.h"
#include "targets.h"

#define BAUD_RATE 9600
#define BIT_TIME (SYS_CLOCK_FREQ_HZ / BAUD_RATE)
#define HALF_BIT_TIME (BIT_TIME / 2)

#define RX_BUF_SIZE 64

static volatile bool tx_active = false;
static volatile uint8_t tx_byte = 0;

static volatile bool rx_active = false;
static volatile uint8_t rx_byte = 0;
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint32_t rx_buf_head = 0;
static volatile uint32_t rx_buf_tail = 0;

static uint8_t rx_buf_write(uint8_t data) {
  const uint32_t next = (rx_buf_head + 1) % RX_BUF_SIZE;
  if (next == rx_buf_tail)
    return 0;

  rx_buf[rx_buf_head] = data;
  rx_buf_head = next;
  return 1;
}

static uint8_t rx_buf_read(uint8_t *data) {
  if (rx_buf_head == rx_buf_tail)
    return 0;

  *data = rx_buf[rx_buf_tail];
  rx_buf_tail = (rx_buf_tail + 1) % RX_BUF_SIZE;
  return 1;
}

static void soft_serial_config_rx() {
  while (tx_active)
    ;

  tmr_interrupt_enable(TMR3, TMR_OVF_INT | TMR_C1_INT | TMR_C2_INT, FALSE);
  tmr_counter_enable(TMR3, FALSE);

  // reset channel config
  TMR3->cval = 0;
  TMR3->c1dt = 0;
  TMR3->c2dt = 0;

  // trigger CH1 on falling edge
  tmr_input_config_type tmr_ic_init;
  tmr_input_default_para_init(&tmr_ic_init);
  tmr_ic_init.input_filter_value = 0;
  tmr_ic_init.input_channel_select = TMR_SELECT_CHANNEL_1;
  tmr_ic_init.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
  tmr_ic_init.input_polarity_select = TMR_INPUT_FALLING_EDGE;
  tmr_input_channel_init(TMR3, &tmr_ic_init, TMR_CHANNEL_INPUT_DIV_1);

  // trigger CH2 on half bit time
  tmr_output_config_type tmr_oc_init;
  tmr_output_default_para_init(&tmr_oc_init);
  tmr_oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_oc_init.oc_idle_state = FALSE;
  tmr_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_oc_init.oc_output_state = FALSE;
  tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_2, &tmr_oc_init);
  tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_2, HALF_BIT_TIME);

  // reset TM3 counter on both edges
  tmr_trigger_input_select(TMR3, TMR_SUB_INPUT_SEL_C1INC);
  tmr_sub_mode_select(TMR3, TMR_SUB_RESET_MODE);
  tmr_sub_sync_mode_set(TMR3, TRUE);

  tmr_counter_enable(TMR3, TRUE);
  tmr_interrupt_enable(TMR3, TMR_C1_INT | TMR_C2_INT, TRUE);

  rx_active = true;
}

static void soft_serial_config_tx() {
  tmr_interrupt_enable(TMR3, TMR_OVF_INT | TMR_C1_INT | TMR_C2_INT, FALSE);
  tmr_counter_enable(TMR3, FALSE);

  TMR3->cval = 0;
  TMR3->c1dt = BIT_TIME + 1;
  TMR3->c2dt = 0;

  TMR3->cctrl_bit.c1en = FALSE;
  TMR3->cm1_output_bit.c1octrl = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  TMR3->cm1_output_bit.c1c = 0;
  TMR3->ctrl2_bit.c1ios = TRUE;
  TMR3->cctrl_bit.c1p = TMR_OUTPUT_ACTIVE_HIGH;
  TMR3->cctrl_bit.c1en = TRUE;

  tmr_trigger_input_select(TMR3, 0);
  tmr_sub_mode_select(TMR3, TMR_SUB_MODE_DIABLE);
  tmr_sub_sync_mode_set(TMR3, FALSE);

  tmr_output_enable(TMR3, TRUE);
  tmr_counter_enable(TMR3, TRUE);
  tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);
}

void soft_serial_init() {
  gpio_mode(IO_PORT, IO_PIN, GPIO_MODE_MUX, GPIO_PULL_UP);
  gpio_pin_mux_config(IO_PORT, IO_PIN_SOURCE, IO_PIN_MUX);

  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 2, 0);

  tmr_base_init(TMR3, BIT_TIME, 0);
  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);

  soft_serial_config_rx();
}

void soft_serial_write_byte(uint8_t val) {
  while (tx_active)
    ;

  if (rx_active) {
    soft_serial_config_tx();
    rx_active = false;
  }
  tx_byte = val;
  tx_active = true;

  soft_serial_config_rx();
}

bool soft_serial_read_byte(uint8_t *data) {
  return rx_buf_read(data);
}

void soft_serial_write(const uint8_t *data, const uint32_t length) {
  for (uint32_t i = 0; i < length; i++) {
    soft_serial_write_byte(data[0]);
  }
}

static void soft_serial_update_tx() {
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) == RESET) {
    return;
  }
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);

  if (!tx_active) {
    return;
  }

  static uint32_t state = 0;

  uint32_t val = BIT_TIME + 1;
  if (state == 0) {
    // start bit
    val = 0;
    state++;
  } else if (state < 9) {
    if ((tx_byte & 0x1) == 0) {
      val = 0;
    }
    tx_byte >>= 1;
    state++;
  } else if (state < 10) {
    // stop bit
    state++;
  } else {
    // done
    state = 0;
    tx_active = false;
    return;
  }
  tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_1, val);
}

static void soft_serial_update_rx() {
  if (!rx_active) {
    return;
  }

  static uint32_t state = 10;

  // falling edge
  if (tmr_flag_get(TMR3, TMR_C1_FLAG)) {
    tmr_flag_clear(TMR3, TMR_C1_FLAG);

    if (state == 10) {
      // start over
      state = 0;
    }
  }

  // half-bit time
  if (tmr_flag_get(TMR3, TMR_C2_FLAG)) {
    tmr_flag_clear(TMR3, TMR_C2_FLAG);

    if (state == 10) {
      return;
    }

    const flag_status level = gpio_input_data_bit_read(IO_PORT, IO_PIN);
    if (state == 0) {
      // start bit
      if (level == RESET) {
        state++;
      } else {
        state = 10;
      }
      rx_byte = 0;
    } else if (state < 9) {
      rx_byte >>= 1;
      if (level) {
        rx_byte |= 0x80;
      }
      state++;
    } else {
      // stop bit
      rx_buf_write(rx_byte);
      state++;
    }
  }
}

void TMR3_GLOBAL_IRQHandler(void) {
  soft_serial_update_tx();
  soft_serial_update_rx();

  tmr_flag_clear(TMR3, TMR_OVF_FLAG | TMR_C1_FLAG | TMR_C2_FLAG);
}