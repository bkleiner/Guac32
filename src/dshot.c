#include "dshot.h"

#include "gpio.h"
#include "mcu.h"
#include "targets.h"

#define DMA_BUFFER_SIZE 32

static volatile bool dshot_update = false;
static volatile uint16_t dshot_value = 0;
static volatile uint16_t dma_buffer[DMA_BUFFER_SIZE];

static void dshot_wait_for_low() {
  while (true) {
    bool was_low = true;
    for (uint32_t i = 0; i < 32; i++) {
      if (gpio_input_data_bit_read(IO_PORT, IO_PIN)) {
        was_low = false;
      }
    }
    if (was_low) {
      return;
    }
  }
}

static void dshot_timer_reset() {
  tmr_reset(TMR3);
  tmr_base_init(TMR3, 0xFFFF, 11);
  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);

  tmr_input_config_type tmr_ic_init;
  tmr_input_default_para_init(&tmr_ic_init);
  tmr_ic_init.input_filter_value = 0;
  tmr_ic_init.input_channel_select = TMR_SELECT_CHANNEL_1;
  tmr_ic_init.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
  tmr_ic_init.input_polarity_select = TMR_INPUT_BOTH_EDGE;
  tmr_input_channel_init(TMR3, &tmr_ic_init, TMR_CHANNEL_INPUT_DIV_1);

  tmr_dma_request_enable(TMR3, TMR_C1_DMA_REQUEST, TRUE);
  tmr_counter_enable(TMR3, TRUE);
}

void dshot_init() {
  gpio_mode(IO_PORT, IO_PIN, GPIO_MODE_MUX, GPIO_PULL_NONE);
  gpio_pin_mux_config(IO_PORT, IO_PIN_SOURCE, IO_PIN_MUX);

  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

  nvic_irq_enable(DMA1_Channel5_4_IRQn, 3, 0);

  dma_init_type dma_conf;
  dma_conf.peripheral_base_addr = (uint32_t)&TMR3->c1dt;
  dma_conf.memory_base_addr = (uint32_t)&dma_buffer;
  dma_conf.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_conf.buffer_size = DMA_BUFFER_SIZE;
  dma_conf.peripheral_inc_enable = FALSE;
  dma_conf.memory_inc_enable = TRUE;
  dma_conf.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_conf.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_conf.loop_mode_enable = FALSE;
  dma_conf.priority = DMA_PRIORITY_VERY_HIGH;
  dma_init(DMA1_CHANNEL4, &dma_conf);
  dma_interrupt_enable(DMA1_CHANNEL4, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL4, DMA_DTERR_INT, TRUE);
  dma_channel_enable(DMA1_CHANNEL4, TRUE);

  dshot_timer_reset();
}

bool dshot_read(uint16_t *val) {
  if (!dshot_update) {
    return false;
  }
  dshot_update = false;
  *val = dshot_value >> 5;
  return true;
}

void DMA1_Channel5_4_IRQHandler(void) {
  dma_flag_clear(DMA1_GL4_FLAG);

  dma_channel_enable(DMA1_CHANNEL4, FALSE);
  dma_data_number_set(DMA1_CHANNEL4, DMA_BUFFER_SIZE);

  const uint16_t frame_time = dma_buffer[31] - dma_buffer[0];
  const uint16_t half_bit_time = frame_time / 32;
  for (uint32_t i = 1; i < 32; i += 2) {
    const uint32_t delta = dma_buffer[i] - dma_buffer[i - 1];
    dshot_value <<= 1;
    if (delta > half_bit_time) {
      dshot_value |= 1;
    }
  }

  const uint16_t value = dshot_value >> 4;
  const uint8_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
  if ((dshot_value & 0x0F) == crc) {
    dshot_update = true;
  } else {
    dshot_wait_for_low();
  }

  dshot_timer_reset();
  dma_channel_enable(DMA1_CHANNEL4, TRUE);
}
