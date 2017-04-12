#include <cores/interrupt.h>
#include <test/timerInterrupt.h>

// LED
#define LED_DDR			DDRB		// Data Direction Register for the LED.
#define LED_PIN			PINB		// Data Input Register for the LED.
#define LED_PINOUT		0x80		// Data Direction Register value (0b10000000).
						// This is arduino pin 13 (PB7).
#define LED_PORT 		PORTB		// Data Output Register for the LED.

// INTERRUPT
#define INT_MASK		TIMSK
#define INT_COMP		OCR0A
#define INT_ENABLE		OCIE0A

// 
timerInterrupt ti = new timerInterrupt(INT_MASK, INT_COMP, INT_ENABLE, LED_PORT);
ti::initialize();				// Init.

// MAIN PROGRAM
int main(void){;;}				// Do nothing. We are testing interrupts.



