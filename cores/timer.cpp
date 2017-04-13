/******************************************************************************
 * Quadcopter-Library-v1
 * TIMER.CPP
 *  
 * This file contains functions for the AVR-timers.
 *
 * @author:	Rob Mertens
 * @version: 	1.0.1 
 * @date: 	14/08/2016
 ******************************************************************************/

#include <timer.h>

/*******************************************************************************
 * Constructor for a 8-bit timer, e.g.: timer0.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer::timer(volatile uint8_t * tccr, volatile uint8_t * tcnt, volatile uint8_t * timsk)
{
	// 8-bit.
	_bitness = 8;
	
	// Timer Count Control Register.
	_tccr = tccr;
	
	// Timer Count register.
	_tcnt = tcnt;
	UNUSED(_tcnth);
	UNUSED(_tcntl);
	
	// Timer Interrupt Mask register.
	_timsk = timsk;
}

/*******************************************************************************
 * Constructor for a 16-bit timer, e.g.: timer1.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer::timer(volatile uint8_t * tccr, volatile uint8_t * tcnth, volatile uint8_t * tcntl, volatile uint8_t * timsk)
{
	// 16-bit.
	_bitness = 16;
	
	// Timer Count Control Register.
	_tccr = tccr;
	
	// Timer Count register.
	UNUSED(_tcnt);
	_tcnth = tcnth;
	_tcntl = tcntl;
	
	// Timer Interrupt Mask register.
	_timsk = timsk;
}

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer.
 ******************************************************************************/
static void timer::initialize(uint8_t prescale, uint8_t mask)
{
	*_tccr = prescale;
	*_timsk = mask;
	
	*_tcnt = 0x00;
}

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer.
 ******************************************************************************/
static void timer::set(uint8_t value)
{
	if(_bitness == 8)
	{
		*_tcnt = value;
	}
	else
	{
		//TODO::wrong notation?!
		*_tcnth = 0x00;
		*_tcntl = 0x00;
	}
}

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer.
 ******************************************************************************/
static void timer::reset()
{
	if(_bitness == 8)
	{
		*_tcnt = 0x00;
	}
	else
	{
		//TODO::wrong notation?!
		*_tcnth = 0x00;
		*_tcntl = 0x00;
	}
}

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer.
 ******************************************************************************/
static void timer::prescaler(uint8_t value)
{
	*_tccr = value;
}

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer.
 * 
 * 
 ******************************************************************************/
uint8_t timer::getCount()
{
	return *_tcnt;
}

/*******************************************************************************
 * Method for obtaining the total summized count since the last reset. Thus
 * overflows are accounted.
 * 
 * REMARK: This method only works if the timer interrupts with an timer_ovf_vect()
 * 
 * @return: _nonResetCount The count value since last reset.
 ******************************************************************************/
uint16_t timer::getNonResetCount()
{
	//TODO::calculations.
	
	return _nonResetCount;
}

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer.
 ******************************************************************************/
uint16_t timer::getOverflow()
{
	return _overflowCount;
}

/*******************************************************************************
 * ISR for the timer class.
 ******************************************************************************/
virtual void timer::interruptServiceRoutine(void)
{
	//TODO::check which interrupt is active.
	_overflowCount++;						// timer_ovf_vect()
	_interruptCount++;						// timer_comp_vect()
}

/*******************************************************************************
 * Enable timer interrupts.
 ******************************************************************************/
virtual void timer::enable(void){}

/*******************************************************************************
 * Disable timer interrupts.
 ******************************************************************************/
virtual void timer::disable(void){}

/*******************************************************************************
 * Clear timer interrupts.
 ******************************************************************************/
virtual void timer::clear(void){}
 
