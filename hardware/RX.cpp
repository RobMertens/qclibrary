/******************************************************************************
 *	Quadcopter-Library-v1
 *  RX.cpp
 *  
 *	This file contains predefined functions for the ESC-class. These functions
 *	serve for driving the motors of the quadcopter. They do this by writing
 *	PWM-signals to the electronic speed controllers (ESC's). Every PWM-signal
 *	takes about 4000 microseconds which directly gives us the refreshrate of 
 * 	the controller:
 *
 *	refreshrate = 1/4000 = 250 Hz.
 *
 *	In this class predefined registers are used. For each hardware-setup
 *	these registers can be different. It is recommended to change the registers
 *	in the DEFINE.h file.
 *
 *  @author Rob Mertens
 *  @version 1.0.1 14/08/2016
 ******************************************************************************/

#include <avr/io.h>
#include <RX.h>

/*******************************************************************************
 * 	Constructor for the ESC-class. By making an object with this constructor
 *	all local variables are set together with the avr-timer.
 ******************************************************************************/
RX::RX(void)
{	
	_pin	= &RX_PIN;								// Pass trough the Pin Input Register to local variable.
	_pcmsk 	= &RX_PCMSK;							// Pass trough the Pin Change Mask Register to local variable.
	
	_pcint	= RX_PCINT;								// Which Pin Change Interrupt Pins are used.
	_pcie	= RX_PCIE;								//
	
	_overflow = 0x0000;
	
	for (uint8_t mask=0x01; mask<=0x80; mask<<=1)	// Loop for determining and splitting the Pin Change Interrupt Pins in different bytes.
    {
    	int n = 0;
    	if(_pcint & mask)
    	{
    		n++;
    		if(n==1)_ch1 |= mask;
    		if(n==2)_ch2 |= mask;
    		if(n==3)_ch3 |= mask;
    		if(n==4)_ch4 |= mask;
    	} 
    }
}

/*******************************************************************************
 * 	Constructor for the ESC-class. By making an object with this constructor
 *	all local variables are set together with the avr-timer.
 ******************************************************************************/
void RX::initialize(void)
{
	cli();											// Disable interrupts before changing the registers.
	PCICR	|= _pcie;								// Set PCICR to enable PCMSKx scan.
	*_pcmsk |= _pcint;								// Set PCINT pins in PCMSKx.
	sei();											// Enable interrupts.
	
	TCCR1B |= ((1 << CS11) | (1 << CS10)); 			// Set up the 16-bit timer prescaler value 64.
													// Maximum possible time for one timer run can be calculated.
													// (maxmicroseconds) = (2^16 - 1) * 4 = 262140us
													// Per overflow flag the overflow word stores one timer value.
													// Maximum possible time with overflows can be calculated.
													// (maxmicroseconds) = (2^16 - 1 + 1) * 262140us = 4h46m46s
}

/*******************************************************************************
 * 	Constructor for the ESC-class. By making an object with this constructor
 *	all local variables are set together with the avr-timer.
 ******************************************************************************/
virtual void RX::interruptServiceRoutine(void)
{
	if(TIFR1 & 0x01) 				// If timer has overflown add an overflow and reset the TOV1-bit.
	{
		_overflow += 0xFF;
		TIFR1 |= 0x01;
	}
	
	if(!(_lastChannel & _ch1) && *_pin & _ch1)
	{
		_channel1 = (TCNT1 + _overflow);
		_lastChannel |= _ch1;
	}
	else if(_lastChannel & _ch1 && !(*_pin & _ch1))
	{
		_channel1 -= (TCNT1 + _overflow);
		_lastChannel &= (_ch1 ^ 0xFF);
	}
	
	if(!(_lastChannel & _ch2) && *_pin & _ch2)
	{
		_channel2  = (TCNT1 + _overflow);
		_lastChannel |= _ch2;
	}
	else if(_lastChannel & _ch2 && !(*_pin & _ch2))
	{
		_channel2 -= (TCNT1 + _overflow);
		_lastChannel &= (_ch2 ^ 0xFF);
	}
	
	if(!(_lastChannel & _ch3) && *_pin & _ch3)
	{
		_channel3  = (TCNT1 + _overflow);
		_lastChannel |= _ch3;
	}
	else if(_lastChannel & _ch3 && !(*_pin & _ch3))
	{
		_channel3 -= (TCNT1 + _overflow);
		_lastChannel &= (_ch3 ^ 0xFF);
	}
	
	if(!(_lastChannel & _ch4) && *_pin & _ch4)
	{
		_channel4  = (TCNT1 + _overflow);
		_lastChannel |= _ch4;
	}
	else if(_lastChannel & _ch4 && !(*_pin & _ch4))
	{	_channel4 -= (TCNT1 + _overflow);
		_lastChannel &= (_ch4 ^ 0xFF);
	}
	
	if(!(_lastChannel & _pcint))	// Not necessary since we have 4h46m46s of time.
	{
		TCNT1 = 0;
		_overflow = 0x0000;
	}
}
