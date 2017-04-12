/******************************************************************************
 * Quadcopter-Library-v1
 * ESC.cpp
 *  
 * This file contains predefined functions for the ESC-class. These functions
 * serve for driving the motors of the quadcopter. They do this by writing
 * PWM-signals to the electronic speed controllers (ESC's). Every PWM-signal
 * takes about 4000 microseconds (HIGH and LOW part) which directly gives us 
 * the refreshrate of the controller:
 * 
 * refreshrate = 1/0.004 = 250 Hz.
 *
 * PLEASE NOTE: This class is based on arduino's which use a clock frequency 
 *		of 16Mhz. Other arduino's will NOT generate a 4000us signal.
 *
 * In this class predefined registers are used. For each hardware-setup
 * these registers can be different. It is recommended to change the registers
 * in the DEFINE.h file.
 *
 * @author: 	Rob Mertens
 * @date:	14/08/2016
 * @version: 	1.0.1
 ******************************************************************************/

#include <avr/io.h>
#include <ESC.h>

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer. This constructor
 * is compatible for most ESC's which have a DEFAULT microsecond PMW-range of
 * minimum 1000us (motors off) and maximum 2000us (motors at maximum throttle).
 * 
 * @param:
 * @param:
 * @param:
 * @param:
 * @param:
 * @param:
 ******************************************************************************/
ESC::ESC(volatile uint8_t * ddr, volatile uint8_t * pin, volatile uint8_t * port, uint8_t mask, int minMicrosecond=1000, int maxMicrosecond=2000)
{	
	_ddr  = &ddr;							// Put definitions in local variables.
	_pin  = &pin;
	_port = &port;

	*_ddr |= mask;
	
	_HIGH = *_ddr;
	_LOW  = *_ddr ^ 0xFF;

	_minTicks = minMicrosecond/4 - 1;
	_maxTicks = maxMicrosecond/4 - 1;

	for (uint8_t mask=0x01; mask<=0x80; mask<<=1)			// Loop through every bit in a byte.
	{
		int n = 0;
		if(*_ddr & mask)
		{
			n++;
			if(n==1)_esc1 |= mask;
			if(n==2)_esc2 |= mask;
			if(n==3)_esc3 |= mask;
			if(n==4)_esc4 |= mask;
		} 
	}
}

/*******************************************************************************
 * Method for initializing the timer-settings.
 ******************************************************************************/
void ESC::initialize()
{
	TCCR0B |= (1 << CS11) | (1 << CS10); 				// Set up the 8-bit timer prescaler value 64.
    									// Thus our timer resolution equals:
    									// (absolute) = 64/16000000 = 4us
    									// (relative) = 4/4000 = 0.1%
}

/*******************************************************************************
 * Method for writing variable PWM-pulses in length to the ESC's.
 *	
  *@param: us1 The pulselength for ESC1 in microseconds.
 * @param: us2 The pulselength for ESC2 in microseconds.
 * @param: us3 The pulselength for ESC3 in microseconds.
 * @param: us4 The pulselength for ESC4 in microseconds.
 ******************************************************************************/
void ESC::writeVariableSpeed(int us1, int us2, int us3, int us4)
{
	*_port |= _HIGH;						// Set ouput HIGH.
	TCNT0 = 0; 							// Reset timer.
	uint16_t timerOverflow = 0x0000;				// Private overflow word.
	
	int tck1 = microsecondsToTicks(us1);				// Transform microseconds to timer compatible ticks.
	int tck2 = microsecondsToTicks(us2);
	int tck3 = microsecondsToTicks(us3);
	int tck4 = microsecondsToTicks(us4);
	
	while((timerOverflow + TCNT0) < 999)
	{
		if(TIFR0 & 0x01)					// Overflow occured
		{
			timerOverflow += 0xFF;				// Add maximum timervalue to overflow word.
			TIFR0 |= 0x01;					// Reset overflow bit.
		}
		
		if((timerOverflow + TCNT0) >= tck1 && *_pin & _esc1)*_port &= (_esc1 ^ 0xFF);
		if((timerOverflow + TCNT0) >= tck2 && *_pin & _esc2)*_port &= (_esc2 ^ 0xFF);
		if((timerOverflow + TCNT0) >= tck3 && *_pin & _esc3)*_port &= (_esc3 ^ 0xFF);
		if((timerOverflow + TCNT0) >= tck4 && *_pin & _esc4)*_port &= (_esc4 ^ 0xFF);
	}
}

/*******************************************************************************
 * Method for writing LOW PWM-pulses to the ESC's.
 ******************************************************************************/
void ESC::writeMinimumSpeed()
{
	*_port |= _HIGH;						// Set ouput HIGH.
	TCNT0 = 0; 							// Reset timer.
	uint16_t timerOverflow = 0x0000;				// Private overflow word.
	
	int tck = _minTicks;						// High pulse takes minimum microseconds.
	
	while((timerOverflow + TCNT0) < 999)				// Total time takes 4000 microseconds.
	{		
		if(TIFR0 & 0x01)					// Overflow occured
		{
			timerOverflow += 0xFF;				// Add maximum timervalue to overflow word.
			TIFR0 |= 0x01;					// Reset overflow bit.
		}
		if((timerOverflow + TCNT0) >= tck && *_pin & _HIGH)*_port &= _LOW;
	}
}

/*******************************************************************************
 * Method for writing HIGH PWM-pulses to the ESC's.
 ******************************************************************************/
void ESC::writeMaximumSpeed()
{
	*_port |= _HIGH;						// Set ouput HIGH.
	TCNT0 = 0; 							// Reset timer
	uint16_t timerOverflow = 0x0000;				// Private overflow word.
	
	int tck = _maxTicks;						// High pulse takes maximum microseconds.
	
	while((timerOverflow + TCNT0) < 999)				// Total time takes 4000 microseconds.
	{		
		if(TIFR0 & 0x01)					// Overflow occured
		{
			timerOverflow += 0xFF;				// Add maximum timervalue to overflow word.
			TIFR0 |= 0x01;					// Reset overflow bit.
		}
		if((timerOverflow + TCNT0) >= tck && *_pin & _HIGH){*_port &= _LOW;}
	}
}

/*******************************************************************************
 * Private method for transforming the a microsecond value to a timer
 * compatible value. It also contains a safety check on minimum and maximum
 * values.
 *
 * @param: us The length of the PWM-pulse in microseconds.
 * @return: tck The lenght of the PWM-pulse in timer compatible ticks.
 ******************************************************************************/
int ESC::microsecondsToTicks(int us)
{
	int tck;
	
	tck = us/4 - 1;							// ticks = (1 / desired frequency) / (prescaler / clock speed) - 1.
	if(tck > _maxTicks)tck = _maxTicks;
	else if(tck < _minTicks)tck = _minTicks;
	
	return tck;
}
