#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  PHASE_AB,
  PHASE_CB,
  PHASE_CA,
  PHASE_BA,
  PHASE_BC,
  PHASE_AC,
} phase_sequence_t;

void phase_init();
bool phase_step(const phase_sequence_t seq, const uint32_t pwm_hz, const uint32_t throttle);