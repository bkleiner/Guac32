#include "phase.h"

#include <stdbool.h>

#include "config.h"
#include "gpio.h"
#include "mcu.h"
#include "targets.h"
#include "util.h"

#define PHASE_A_IF TMR_C3_INT
#define PHASE_B_IF TMR_C2_INT
#define PHASE_C_IF TMR_C1_INT

#ifdef USE_INVERTED_HIGH
#define HIGH_ENABLE_STATE TMR_OUTPUT_ACTIVE_LOW
#define HIGH_DISABLE_STATE TMR_OUTPUT_ACTIVE_HIGH
#else
#define HIGH_ENABLE_STATE TMR_OUTPUT_ACTIVE_HIGH
#define HIGH_DISABLE_STATE TMR_OUTPUT_ACTIVE_LOW
#endif

#ifdef USE_INVERTED_LOW
#define LOW_ENABLE_STATE TMR_OUTPUT_ACTIVE_LOW
#define LOW_DISABLE_STATE TMR_OUTPUT_ACTIVE_HIGH
#else
#define LOW_ENABLE_STATE TMR_OUTPUT_ACTIVE_HIGH
#define LOW_DISABLE_STATE TMR_OUTPUT_ACTIVE_LOW
#endif

static void phase_a_config(const uint32_t mode, const bool enable_low) {
  TMR1->cctrl_bit.c3cp = enable_low ? LOW_ENABLE_STATE : LOW_DISABLE_STATE;
  TMR1->ctrl2_bit.c3cios = enable_low ? LOW_ENABLE_STATE : LOW_DISABLE_STATE;
  TMR1->cm2_output_bit.c3octrl = mode;
}

static void phase_b_config(const uint32_t mode, const bool enable_low) {
  TMR1->cctrl_bit.c2cp = enable_low ? LOW_ENABLE_STATE : LOW_DISABLE_STATE;
  TMR1->ctrl2_bit.c2cios = enable_low ? LOW_ENABLE_STATE : LOW_DISABLE_STATE;
  TMR1->cm1_output_bit.c2octrl = mode;
}

static void phase_c_config(const uint32_t mode, const bool enable_low) {
  TMR1->cctrl_bit.c1cp = enable_low ? LOW_ENABLE_STATE : LOW_DISABLE_STATE;
  TMR1->ctrl2_bit.c1cios = enable_low ? LOW_ENABLE_STATE : LOW_DISABLE_STATE;
  TMR1->cm1_output_bit.c1octrl = mode;
}

static void phase_exti_pol(uint32_t line, exint_polarity_config_type pol) {
  EXINT->polcfg1 &= ~line;
  EXINT->polcfg2 &= ~line;
  if (pol == EXINT_TRIGGER_RISING_EDGE) {
    EXINT->polcfg1 |= line;
  } else if (pol == EXINT_TRIGGER_FALLING_EDGE) {
    EXINT->polcfg2 |= line;
  } else {
    EXINT->polcfg1 |= line;
    EXINT->polcfg2 |= line;
  }
}

void phase_init() {
  crm_periph_clock_enable(CRM_CMP_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  TMR1->pr = (SYS_CLOCK_FREQ_HZ / PHASE_PWM_MIN) - 1;
  TMR1->div = 0;

  TMR1->ctrl1_bit.cnt_dir = TMR_COUNT_UP;
  TMR1->ctrl1_bit.clkdiv = TMR_CLOCK_DIV1;
  TMR1->ctrl1_bit.prben = FALSE;

  // PHASE C: channel 1
  gpio_mode(PHASE_C_GPIO_PORT_HIGH, PHASE_C_GPIO_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE);
  gpio_pin_mux_config(PHASE_C_GPIO_PORT_HIGH, PHASE_C_PIN_SOURCE_HIGH, GPIO_MUX_2);
  gpio_mode(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE);
  gpio_pin_mux_config(PHASE_C_GPIO_PORT_LOW, PHASE_C_PIN_SOURCE_LOW, GPIO_MUX_2);

  TMR1->cm1_output_bit.c1c = 0; // ouput
  TMR1->cm1_output_bit.c1oien = FALSE;
  TMR1->cm1_output_bit.c1oben = FALSE;
  TMR1->cm1_output_bit.c1osen = FALSE;
  TMR1->cctrl_bit.c1p = HIGH_ENABLE_STATE;
  TMR1->ctrl2_bit.c1ios = HIGH_ENABLE_STATE;
  TMR1->cctrl_bit.c1en = TRUE;
  TMR1->cctrl_bit.c1cp = LOW_ENABLE_STATE;
  TMR1->ctrl2_bit.c1cios = LOW_ENABLE_STATE;
  TMR1->cctrl_bit.c1cen = TRUE;
  phase_c_config(TMR_OUTPUT_CONTROL_FORCE_LOW, true);

  // PHASE B: channel 2
  gpio_mode(PHASE_B_GPIO_PORT_HIGH, PHASE_B_GPIO_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE);
  gpio_pin_mux_config(PHASE_B_GPIO_PORT_HIGH, PHASE_B_PIN_SOURCE_HIGH, GPIO_MUX_2);
  gpio_mode(PHASE_B_GPIO_PORT_LOW, PHASE_B_GPIO_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE);
  gpio_pin_mux_config(PHASE_B_GPIO_PORT_LOW, PHASE_B_PIN_SOURCE_LOW, GPIO_MUX_2);

  TMR1->cm1_output_bit.c2c = 0; // ouput
  TMR1->cm1_output_bit.c2oien = FALSE;
  TMR1->cm1_output_bit.c2oben = FALSE;
  TMR1->cm1_output_bit.c2osen = FALSE;
  TMR1->cctrl_bit.c2p = HIGH_ENABLE_STATE;
  TMR1->ctrl2_bit.c2ios = HIGH_ENABLE_STATE;
  TMR1->cctrl_bit.c2en = TRUE;
  TMR1->cctrl_bit.c2cp = LOW_ENABLE_STATE;
  TMR1->ctrl2_bit.c2cios = LOW_ENABLE_STATE;
  TMR1->cctrl_bit.c2cen = TRUE;
  phase_b_config(TMR_OUTPUT_CONTROL_FORCE_LOW, true);

  // PHASE A: channel 3
  gpio_mode(PHASE_A_GPIO_PORT_HIGH, PHASE_A_GPIO_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE);
  gpio_pin_mux_config(PHASE_A_GPIO_PORT_HIGH, PHASE_A_PIN_SOURCE_HIGH, GPIO_MUX_2);
  gpio_mode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE);
  gpio_pin_mux_config(PHASE_A_GPIO_PORT_LOW, PHASE_A_PIN_SOURCE_LOW, GPIO_MUX_2);

  TMR1->cm2_output_bit.c3c = 0; // ouput
  TMR1->cm2_output_bit.c3oien = FALSE;
  TMR1->cm2_output_bit.c3oben = FALSE;
  TMR1->cm2_output_bit.c3osen = FALSE;
  TMR1->cctrl_bit.c3p = HIGH_ENABLE_STATE;
  TMR1->ctrl2_bit.c3ios = HIGH_ENABLE_STATE;
  TMR1->cctrl_bit.c3en = TRUE;
  TMR1->cctrl_bit.c3cp = LOW_ENABLE_STATE;
  TMR1->ctrl2_bit.c3cios = LOW_ENABLE_STATE;
  TMR1->cctrl_bit.c3cen = TRUE;
  phase_a_config(TMR_OUTPUT_CONTROL_FORCE_LOW, true);

  // start timer
  TMR1->brk_bit.dtc = DEAD_TIME;
  TMR1->brk_bit.oen = TRUE;

  TMR1->ctrl1_bit.tmren = TRUE;
}

bool phase_step(const phase_sequence_t seq, const uint32_t period, const uint32_t duty) {
  bool rising = true;

  TMR1->pr = period - 1;

  TMR1->c3dt = duty;
  TMR1->c2dt = duty;
  TMR1->c1dt = duty;

  switch (seq) {
  case PHASE_AB: {
    phase_a_config(TMR_OUTPUT_CONTROL_PWM_MODE_A, true); // pwm
    phase_b_config(TMR_OUTPUT_CONTROL_FORCE_LOW, true);  // low
    phase_c_config(TMR_OUTPUT_CONTROL_FORCE_LOW, false); // float
    CMP->ctrlsts = (CMP_POL_NON_INVERTING << 15) | (CMP_NON_INVERTING_PA1 << 7) | (PHASE_C_COMP << 4) | 0x1;
    phase_exti_pol(EXINT_LINE_21, EXINT_TRIGGER_FALLING_EDGE);
    rising = false;
    break;
  }
  case PHASE_CB: {
    phase_c_config(TMR_OUTPUT_CONTROL_PWM_MODE_A, true); // pwm
    phase_b_config(TMR_OUTPUT_CONTROL_FORCE_LOW, true);  // low
    phase_a_config(TMR_OUTPUT_CONTROL_FORCE_LOW, false); // float
    CMP->ctrlsts = (CMP_POL_NON_INVERTING << 15) | (CMP_NON_INVERTING_PA1 << 7) | (PHASE_A_COMP << 4) | 0x1;
    phase_exti_pol(EXINT_LINE_21, EXINT_TRIGGER_RISING_EDGE);
    rising = true;
    break;
  }
  case PHASE_CA: {
    phase_c_config(TMR_OUTPUT_CONTROL_PWM_MODE_A, true); // pwm
    phase_a_config(TMR_OUTPUT_CONTROL_FORCE_LOW, true);  // low
    phase_b_config(TMR_OUTPUT_CONTROL_FORCE_LOW, false); // float
    CMP->ctrlsts = (CMP_POL_NON_INVERTING << 15) | (CMP_NON_INVERTING_PA1 << 7) | (PHASE_B_COMP << 4) | 0x1;
    phase_exti_pol(EXINT_LINE_21, EXINT_TRIGGER_FALLING_EDGE);
    rising = false;
    break;
  }
  case PHASE_BA: {
    phase_b_config(TMR_OUTPUT_CONTROL_PWM_MODE_A, true); // pwm
    phase_a_config(TMR_OUTPUT_CONTROL_FORCE_LOW, true);  // low
    phase_c_config(TMR_OUTPUT_CONTROL_FORCE_LOW, false); // float
    CMP->ctrlsts = (CMP_POL_NON_INVERTING << 15) | (CMP_NON_INVERTING_PA1 << 7) | (PHASE_C_COMP << 4) | 0x1;
    phase_exti_pol(EXINT_LINE_21, EXINT_TRIGGER_RISING_EDGE);
    rising = true;
    break;
  }
  case PHASE_BC: {
    phase_b_config(TMR_OUTPUT_CONTROL_PWM_MODE_A, true); // pwm
    phase_c_config(TMR_OUTPUT_CONTROL_FORCE_LOW, true);  // low
    phase_a_config(TMR_OUTPUT_CONTROL_FORCE_LOW, false); // float
    CMP->ctrlsts = (CMP_POL_NON_INVERTING << 15) | (CMP_NON_INVERTING_PA1 << 7) | (PHASE_A_COMP << 4) | 0x1;
    phase_exti_pol(EXINT_LINE_21, EXINT_TRIGGER_FALLING_EDGE);
    rising = false;
    break;
  }
  case PHASE_AC: {
    phase_a_config(TMR_OUTPUT_CONTROL_PWM_MODE_A, true); // pwm
    phase_c_config(TMR_OUTPUT_CONTROL_FORCE_LOW, true);  // low
    phase_b_config(TMR_OUTPUT_CONTROL_FORCE_LOW, false); // float
    CMP->ctrlsts = (CMP_POL_NON_INVERTING << 15) | (CMP_NON_INVERTING_PA1 << 7) | (PHASE_B_COMP << 4) | 0x1;
    phase_exti_pol(EXINT_LINE_21, EXINT_TRIGGER_RISING_EDGE);
    rising = true;
    break;
  }
  }

  return rising;
}