/******************************************************************************
 * Quadcopter-Library-v1
 * timerInterrupt.cpp
 *  
 * Test for <in-class> timer compare interrupts.
 * 
 * @author: Rob Mertens
 ******************************************************************************/

#include <avr/io.h>
#include <timerInterrupt.h>

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer.
 ******************************************************************************/
timerInterrupt::timerInterrupt(volatile uint8_t * TIMSK, volatile uint8_t * OCR, volatile uint8_t * OCIE, volatile uint8_t * LED)
{	
	_timsk 	= TIMSK;				// Pass through the Pin Change Mask Register to local variable.
	_ocr	= OCR;					// Pass through the Pin Input Register to local variable.
	_ocie	= OCIE;					// Pass through the Output Compare Interrupt Enable register.
	_led	= LED;
}

/*******************************************************************************
 * Initialization.
 ******************************************************************************/
void timerInterrupt::initialize(uint8_t timerCompareValue)
{
	cli();						// Disable interrupts before changing the registers.
	*_ocr = timerCompareValue;			// Assign the timer compare value.
	*_timsk |= _BV(*_ocie);				// Enable timer compare interrupts with specific OCIE byte.
	sei();						// Enable interrupts.
}

/*******************************************************************************
 * ISR.
 ******************************************************************************/
virtual void timerInterrupt::interruptServiceRoutine(void)
{
	*_led ^= 0xF0;					// Led on/off.
}

/*******************************************************************************
 * Enable timer compare interrupts.
 ******************************************************************************/
virtual void timerInterrupt::enable(void){}

/*******************************************************************************
 * Disable timer compare interrupts.
 ******************************************************************************/
virtual void timerInterrupt::disable(void){}

/*******************************************************************************
 * Clear timer compare interrupts.
 ******************************************************************************/
virtual void timerInterrupt::clear(void){}
