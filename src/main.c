#include "mcu.h"

#include <stdlib.h>

#include "config.h"
#include "dshot.h"
#include "gpio.h"
#include "phase.h"
#include "serial.h"
#include "targets.h"
#include "util.h"

typedef enum {
  MOTOR_OFF,
  MOTOR_STARTUP_FORCED,
  MOTOR_STARTUP_FREE,
  MOTOR_RUNNING,
} motor_state_t;

static uint32_t throttle = 0;
static uint32_t advance = 30;
static uint32_t filter = BEMF_FILTER_MAX;

static phase_sequence_t phase_seq = PHASE_AB;
static uint32_t pwm_duty = 0;
static uint32_t pwm_period = SYS_CLOCK_FREQ_HZ / PHASE_PWM_MIN;

static volatile motor_state_t motor_state = MOTOR_OFF;
static volatile uint32_t rpm = 0;
static volatile uint32_t erpm = 0;
static volatile uint32_t interval = 0;
static volatile uint32_t startup_counter = 0;
static volatile uint32_t accel = 0;
static volatile uint32_t phase_periods[6];
static volatile bool rising = true;

static void phase_advance() {
  phase_seq = (phase_seq + 1) % 6;
  rising = phase_step(phase_seq, pwm_period, pwm_duty);
}

int main(void) {
  system_clock_config();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_2);

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  serial_init();
  dshot_init();
  phase_init();

  crm_periph_clock_enable(CRM_TMR16_PERIPH_CLOCK, TRUE);
  TMR16->pr = 5000;
  TMR16->div = 59;
  TMR16->ctrl1_bit.cnt_dir = TMR_COUNT_UP;
  TMR16->ctrl1_bit.clkdiv = TMR_CLOCK_DIV1;
  TMR16->ctrl1_bit.prben = TRUE;
  TMR16->ctrl1_bit.tmren = TRUE;
  nvic_irq_enable(TMR16_GLOBAL_IRQn, 1, 0);

  crm_periph_clock_enable(CRM_TMR17_PERIPH_CLOCK, TRUE);
  TMR17->pr = 0xFFFF;
  TMR17->div = 59;
  TMR17->ctrl1_bit.cnt_dir = TMR_COUNT_UP;
  TMR17->ctrl1_bit.clkdiv = TMR_CLOCK_DIV1;
  TMR17->ctrl1_bit.prben = TRUE;
  TMR17->ctrl1_bit.tmren = TRUE;

  nvic_irq_enable(ADC1_CMP_IRQn, 0, 0);

  static uint32_t counter = 0;
  static uint32_t last_throttle = 0;
  while (1) {
    uint16_t val = 0;
    if (dshot_read(&val)) {
      if (val > 47) {
        throttle = constrain((val - 47), 1, 2000);
      } else if (val == 0) {
        throttle = 0;
      }
    }

    if (throttle == 0) {
      motor_state = MOTOR_OFF;
    } else if (last_throttle == 0) {
      TMR16->pr = 5000;
      tmr_interrupt_enable(TMR16, TMR_OVF_INT, TRUE);
      motor_state = MOTOR_STARTUP_FORCED;
    } else if (throttle > last_throttle) {
      accel = 1;
    } else {
      accel = 0;
    }

    switch (motor_state) {
    case MOTOR_OFF:
      interval = 0;
      startup_counter = 0;
      rpm = 0;
      erpm = 0;
      pwm_duty = 0;
      pwm_period = SYS_CLOCK_FREQ_HZ / PHASE_PWM_MIN;
      phase_periods[0] = phase_periods[1] = phase_periods[2] = phase_periods[3] = phase_periods[4] = phase_periods[5] = 0;

      tmr_interrupt_enable(TMR16, TMR_OVF_INT, FALSE);
      phase_step(phase_seq, pwm_period, 0);
      break;

    case MOTOR_STARTUP_FORCED:
    case MOTOR_STARTUP_FREE: {
      if (throttle != last_throttle) {
        const uint32_t throttle_startup = constrain(throttle, STARTUP_THROTTLE_MIN, STARTUP_THROTTLE_MAX);
        const uint32_t pwm_hz = map(throttle_startup, THROTTLE_MAX * 0.2, THROTTLE_MAX * 0.8, PHASE_PWM_MIN, PHASE_PWM_MAX);
        pwm_period = (SYS_CLOCK_FREQ_HZ / pwm_hz);
        pwm_duty = map(throttle_startup, 1, THROTTLE_MAX, DEAD_TIME + pwm_period / 10, pwm_period);
      }
      filter = BEMF_FILTER_MAX * 2;
      break;
    }

    case MOTOR_RUNNING: {
      const uint32_t period = (phase_periods[0] + phase_periods[1] + phase_periods[2] + phase_periods[3] + phase_periods[4] + phase_periods[5]) / 6;
      erpm = 6000000 / (period / 2);
      rpm = erpm / 12;

      if (throttle != last_throttle) {
        const uint32_t pwm_hz = map(throttle, THROTTLE_MAX * 0.2, THROTTLE_MAX * 0.8, PHASE_PWM_MIN, PHASE_PWM_MAX);
        pwm_period = (SYS_CLOCK_FREQ_HZ / pwm_hz);
        pwm_duty = map(throttle, 1, THROTTLE_MAX, DEAD_TIME + pwm_period / 10, pwm_period);
      }
      filter = map(interval, 90, 200, BEMF_FILTER_MIN, BEMF_FILTER_MAX);
      break;
    }
    }

    if ((counter % 10000) == 0) {
      serial_printf("%d,%d,%d\r\n", throttle, interval, rpm);
    }

    last_throttle = throttle;
    counter++;
  }
}

void ADC1_CMP_IRQHandler(void) {
  const uint16_t delta = TMR17->cval;

  if (exint_flag_get(EXINT_LINE_21) == RESET) {
    return;
  }
  exint_flag_clear(EXINT_LINE_21);

  const uint32_t value = rising ? SET : RESET;
  for (uint32_t i = 0; i < filter; i++) {
    if (CMP->ctrlsts_bit.cmpvalue != value) {
      return;
    }
  }

  exint_interrupt_enable(EXINT_LINE_21, FALSE);

  const uint16_t pr = max((interval / 12) * (advance / 5), 2);

  switch (motor_state) {
  case MOTOR_STARTUP_FORCED:
    TMR16->pr = 5000;

    if (delta < 4900 || delta > 5100) {
      startup_counter = 0;
    } else if (startup_counter++ >= 10) {
      motor_state = MOTOR_STARTUP_FREE;
    }
    break;

  case MOTOR_STARTUP_FREE:
    TMR16->pr = pr;

    if (delta > (interval + 1000)) {
      motor_state = MOTOR_STARTUP_FORCED;
      startup_counter = 0;
    } else if (startup_counter++ >= 100) {
      motor_state = MOTOR_RUNNING;
    }
    break;

  case MOTOR_RUNNING: {
    TMR16->pr = pr >> accel;
    break;
  }

  default:
    break;
  }

  phase_periods[phase_seq] = interval = ((3 * interval) + delta) / 4;

  const uint16_t cval = max((uint16_t)(TMR17->cval) - delta, 0);
  TMR16->cval = cval;
  TMR17->cval = cval;

  tmr_flag_clear(TMR16, TMR_OVF_FLAG);
  tmr_interrupt_enable(TMR16, TMR_OVF_INT, TRUE);
}

void TMR16_GLOBAL_IRQHandler(void) {
  if (tmr_flag_get(TMR16, TMR_OVF_FLAG) == RESET) {
    return;
  }
  tmr_flag_clear(TMR16, TMR_OVF_FLAG);

  if (motor_state == MOTOR_STARTUP_FREE || motor_state == MOTOR_RUNNING) {
    tmr_interrupt_enable(TMR16, TMR_OVF_INT, FALSE);
  }
  phase_advance();
  exint_interrupt_enable(EXINT_LINE_21, TRUE);
}
