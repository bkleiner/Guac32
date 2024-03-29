#pragma once

#include "mcu.h"

#ifdef NEUTRONRC_IH_F421

#define DEAD_TIME 60

#define IO_PORT GPIOB
#define IO_PIN GPIO_PINS_4
#define IO_PIN_SOURCE GPIO_PINS_SOURCE4
#define IO_PIN_MUX GPIO_MUX_1

#define TELE_PORT GPIOB
#define TELE_PIN GPIO_PINS_6
#define TELE_PIN_SOURCE GPIO_PINS_SOURCE6
#define TELE_PIN_MUX GPIO_MUX_0

#define USE_INVERTED_HIGH

#define PHASE_A_COMP CMP_INVERTING_PA0
#define PHASE_A_GPIO_PORT_LOW GPIOB
#define PHASE_A_GPIO_LOW GPIO_PINS_1
#define PHASE_A_PIN_SOURCE_LOW GPIO_PINS_SOURCE1
#define PHASE_A_GPIO_PORT_HIGH GPIOA
#define PHASE_A_GPIO_HIGH GPIO_PINS_10
#define PHASE_A_PIN_SOURCE_HIGH GPIO_PINS_SOURCE10

#define PHASE_B_COMP CMP_INVERTING_PA4
#define PHASE_B_GPIO_PORT_LOW GPIOB
#define PHASE_B_GPIO_LOW GPIO_PINS_0
#define PHASE_B_PIN_SOURCE_LOW GPIO_PINS_SOURCE0
#define PHASE_B_GPIO_PORT_HIGH GPIOA
#define PHASE_B_GPIO_HIGH GPIO_PINS_9
#define PHASE_B_PIN_SOURCE_HIGH GPIO_PINS_SOURCE9

#define PHASE_C_COMP CMP_INVERTING_PA5
#define PHASE_C_GPIO_PORT_LOW GPIOA
#define PHASE_C_GPIO_LOW GPIO_PINS_7
#define PHASE_C_PIN_SOURCE_LOW GPIO_PINS_SOURCE7
#define PHASE_C_GPIO_PORT_HIGH GPIOA
#define PHASE_C_GPIO_HIGH GPIO_PINS_8
#define PHASE_C_PIN_SOURCE_HIGH GPIO_PINS_SOURCE8

#endif