#ifndef PERIPH_H_
#define PERIPH_H_

#include <stm32f3xx.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>

// Begin C++ extern to C
#ifdef __cplusplus
extern "C" {
#endif

// Pin/port configuration for ioSetDirection
// General-purpose input
#define DDR_INPUT 0x00
// Input pull-up
#define DDR_INPUT_PULLUP 0x10
// Input pull-down
#define DDR_INPUT_PULLDOWN 0x20
// Analog input
#define DDR_INPUT_ANALOG 0x03
// General-purpose output
#define DDR_OUTPUT 0x01
// Open-drain output
#define DDR_OUTPUT_OD 0x05
// Open-drain output with pull up
#define DDR_OUTPUT_ODPU 0x15
// Alternate function I/O
#define DDR_AFO 0x02
// Alternate function I/O with pull up
#define DDR_AFO_PU 0x12
// Alternate function open-drain I/O
#define DDR_AFO_OD 0x06
// Alternate function open-drain I/O with pull up
#define DDR_AFO_ODPU 0x16

// Gets the digital value (1 or 0) of a pin configured as a digital input
__STATIC_FORCEINLINE bool ioGetInput(GPIO_TypeDef* port, uint32_t pin) {
	const uint32_t mask = (uint32_t)(0x00000001 << (pin & 0x0F));
	return (port->IDR & mask) != 0;
}

// Sets the digital value (1 or 0) of a pin configured as a digital output
__STATIC_FORCEINLINE void ioSetOutput(GPIO_TypeDef* port, uint32_t pin, bool value) {
	const uint32_t mask = (uint32_t)(0x00000001 << (pin & 0x0F));
	if (value)
		// Atomic bit set
		port->BSRR = mask;
	else
		// Atomic bit reset
		port->BRR = mask;
}

// Configures the AFIO assignment for the specified pin (assignment table is in part datasheet)
__STATIC_FORCEINLINE void ioSetAlternateFunction(GPIO_TypeDef *port, uint32_t pin, uint8_t afio) {
	const uint32_t idx = (pin & 0x07) << 2, bank = (pin < 8) ? 0 : 1;
	// Load the AFIO bits into the low or high port selection register
	port->AFR[bank] = (port->AFR[bank] & (uint32_t)~(0xF << idx)) | ((uint32_t)(afio &
		0x07) << idx);
}

// Configures the pin as an input or output with a variety of settings
__STATIC_FORCEINLINE void ioSetDirection(GPIO_TypeDef* port, uint32_t pin, uint32_t type) {
	// Force pin in range
	pin &= 0x0F;
	const uint32_t pin2 = pin << 1;
	// Mode has the low 2 bits
	port->MODER = (port->MODER & (uint32_t)~(0x3 << pin2)) | (uint32_t)((type & 0x03) << pin2);
	// Set open drain bit according to 4s position
	if (type & 0x04)
		port->OTYPER |= (uint32_t)(1 << pin);
	else
		port->OTYPER &= (uint32_t)~(1 << pin);
	// PU/PD has the highest 2 bits
	port->PUPDR = (port->PUPDR & (uint32_t)~(0x3 << pin2)) | (uint32_t)(((type & 0x30) >>
		4) << pin2);
}

// End C++ extern to C
#ifdef __cplusplus
}
#endif

#endif
