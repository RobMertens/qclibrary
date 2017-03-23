/******************************************************************************
 *	Quadcopter-Library-v1
 *  TIMER.CPP
 *  
 *	This file contains functions for the AVR-timers.
 *
 *  @author 	Rob Mertens
 *  @version 	1.0.1 
 *	@date 		14/08/2016
 ******************************************************************************/

#include <timer.h>

/*******************************************************************************
 * 	Constructor for the ESC-class. By making an object with this constructor
 *	all local variables are set together with the avr-timer.
 ******************************************************************************/
timer::timer(void)
{

}

/*******************************************************************************
 * 	Constructor for the ESC-class. By making an object with this constructor
 *	all local variables are set together with the avr-timer.
 ******************************************************************************/
timer::timer(volatile uint8_t tccr, volatile uint16_t tcnt)
{
	*_tccr = &tccr;
	*_tcnt = &tcnt;

}

/*******************************************************************************
 * 	Constructor for the ESC-class. By making an object with this constructor
 *	all local variables are set together with the avr-timer.
 ******************************************************************************/
static void timer::set(uint16_t value)
{
	*_tcnt = value;
}

/*******************************************************************************
 * 	Constructor for the ESC-class. By making an object with this constructor
 *	all local variables are set together with the avr-timer.
 ******************************************************************************/
static void timer::reset(void)
{
	*_tcnt = (uint16_t)0;
}

/*******************************************************************************
 * 	Constructor for the ESC-class. By making an object with this constructor
 *	all local variables are set together with the avr-timer.
 ******************************************************************************/
static void setPrescaler(volatile uint8_t tccr)
{
	*_tccr = &tccr;
}

/*******************************************************************************
 * 	Constructor for the ESC-class. By making an object with this constructor
 *	all local variables are set together with the avr-timer.
 ******************************************************************************/
uint16_t timer::getCount(void)
{
	return *_tcnt;
}

/*******************************************************************************
 * 	Constructor for the ESC-class. By making an object with this constructor
 *	all local variables are set together with the avr-timer.
 ******************************************************************************/
uint16_t timer::getOverflow(void)
{
	return _overflow;
}

/*******************************************************************************
 * 	ISR for the timer class.
 ******************************************************************************/
virtual void timer::interruptServiceRoutine(void)
{
	_overflow++;
}
 