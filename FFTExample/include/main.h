#ifndef MAIN_H_
#define MAIN_H_

#include "periph.h"
#include "arm_math.h"

#define PIN_LED_1 GPIOB, 0
#define PIN_LED_2 GPIOB, 7
#define PIN_LED_3 GPIOB, 14

#define ISR_Handler void __attribute__ (( interrupt("IRQ") ))

#define ADC_BUFFER_SIZE 1024

extern uint16_t adcResults[];

#endif
