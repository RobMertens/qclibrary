/******************************************************************************
 * Quadcopter-Library-v1
 * RX.cpp
 *  
 * This file contains functions for the receiver (RX). The signals from the
 * transmitter are measured based on pin change interrupts.
 *
 * TODO::transmitter modes.
 * TODO::channel mapping functions.
 * TODO::calibration functions.
 * TODO::duty cycle calculation.
 * TODO::channel 5 possibility.
 *
 * @author Rob Mertens
 * @version 1.1.1 14/08/2016
 ******************************************************************************/

#include "RX.h"

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer.
 ******************************************************************************/
RX::RX(volatile uint8_t * pin, volatile uint8_t pcmsk, uint8_t pcint, uint8_t pcie, uint16_t periodMicrosecond, uint16_t maxMicrosecond, uint16_t minMicrosecond)
{	
	_pin	= pin;								// Pass trough the Pin Input Register to local variable.
	_pcmsk 	= pcmsk;							// Pass trough the Pin Change Mask Register to local variable.
	
	_pcint	= pcint;							// Which Pin Change Interrupt Pins are used.
	_pcie	= pcie;								//
	
	_t2 	= timer8(t_alias::T2);						// TIMER2.
	
	_periodMicroseconds = periodMicroseconds;
	_maxRxCycle = maxMicrosecond/periodMicroseconds;
	_minRxCycle = minMicrosecond/periodMicroseconds;
	
	for (uint8_t mask=0x01; mask<=0x80; mask<<=1)				// Loop for determining and splitting the Pin Change Interrupt Pins in different bytes.
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
 * Method for initializing the receiver.
 ******************************************************************************/
void RX::initialize(void)
{
	cli();									// Disable interrupts before changing the registers.
	PCICR	|= _pcie;							// Set PCICR to enable PCMSKx scan.
	*_pcmsk |= _pcint;							// Set PCINT pins in PCMSKx.
	sei();									// Enable interrupts.
	
	_t2.setMode(t_mode::NORMAL, t_interrupt::OVF);				// Set up the 8-bit timer prescaler value 64.
	_t2.setPrescaler(1); 							// Maximum possible time for one timer run can be calculated.
	_t2.reset();								// (timer_max) = (2^8 - 1) * (prescale / 16M) = 15,9375us
										// Per overflow flag the overflow word stores one timer value.
										// Maximum possible time with overflows can be calculated.
										// (ovf_max) = (2^32) * 15,9375us = 68451s = 19h00m51s
										// The drone will never fly 19 hours, so OK.
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
float getThrottleChannel()
{
	float rx = 0.0f;
	float dc = 0.0f;
	
	rx = _channel1/_periodMicroseconds;
	dc = rxc2dc(rx);
	
	return dc;
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
float getPitchChannel()
{
	float rx = 0.0f;
	float dc = 0.0f;
	
	rx = _channel3/_periodMicroseconds;
	dc = rxc2dc(rx);
	
	return dc;

}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
float getRollChannel()
{
	float rx = 0.0f;
	float dc = 0.0f;
	
	rx = _channel4/_periodMicroseconds;
	dc = rxc2dc(rx);
	
	return dc;

}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
float getYawChannel()
{
	float rx = 0.0f;
	float dc = 0.0f;
	
	rx = _channel2/_periodMicroseconds;
	dc = rxc2dc(rx);
	
	return dc;

}

/*******************************************************************************
 * Method for mapping RX pulses 2 duty cycle.
 ******************************************************************************/
float ESC::rxc2dc(float rxCycle)
{
	float dc;
	
	if(rxCycle >= _maxRxCycle)dc=1.0f;
	else if(rxCycle <= _minRxCycle)esc=0.0f;
	else{dc = ((1)/(_maxEscCycle - _minEscCycle))*rxCycle}
	
	return dc;
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
uint8_t getExtraChannel(uint8_t channel)
{
	//TODO::
	
	return 0x00;
}

/*******************************************************************************
 * ISR for the receiver class.
 ******************************************************************************/
void RX::interruptServiceRoutine(void)
{	
	if(!(_lastChannel & _ch1) and *_pin & _ch1)
	{
		_channel1 = t2.getNonResetCount();
		_lastChannel |= _ch1;
	}
	else if(_lastChannel & _ch1 and !(*_pin & _ch1))
	{
		_channel1 -= t2.getNonResetCount();
		_lastChannel &= (_ch1 ^ 0xFF);
	}
	
	if(!(_lastChannel & _ch2) and *_pin & _ch2)
	{
		_channel2  = t2.getNonResetCount();
		_lastChannel |= _ch2;
	}
	else if(_lastChannel & _ch2 and !(*_pin & _ch2))
	{
		_channel2 -= t2.getNonResetCount();
		_lastChannel &= (_ch2 ^ 0xFF);
	}
	
	if(!(_lastChannel & _ch3) and *_pin & _ch3)
	{
		_channel3  = t2.getNonResetCount();
		_lastChannel |= _ch3;
	}
	else if(_lastChannel & _ch3 and !(*_pin & _ch3))
	{
		_channel3 -= t2.getNonResetCount();
		_lastChannel &= (_ch3 ^ 0xFF);
	}
	
	if(!(_lastChannel & _ch4) and *_pin & _ch4)
	{
		_channel4  = t2.getNonResetCount();
		_lastChannel |= _ch4;
	}
	else if(_lastChannel & _ch4 and !(*_pin & _ch4))
	{	_channel4 -= t2.getNonResetCount();
		_lastChannel &= (_ch4 ^ 0xFF);
	}
}

/*******************************************************************************
 * Enable pinchange interrupts.
 ******************************************************************************/
void RX::enable(void)
{
	//TODO::
}

/*******************************************************************************
 * Disable pinchange interrupts.
 ******************************************************************************/
void RX::disable(void)
{
	//TODO::	
}

/*******************************************************************************
 * Clear pinchange interrupts.
 ******************************************************************************/
void RX::clear(void)
{
	//TODO::
}

/*******************************************************************************
 * Global forward declaration.
 ******************************************************************************/
RX * RX::_RX[4] = {};

/*******************************************************************************
 * _vector__x -> ISR().
 * 
 * TODO::different avrs.
 * 
 * Call from self.
 ******************************************************************************/
#define RX_ISR(p)								\
ISR(PCINT ## p ## _vect)							\
{										\
	if(RX::_RX[p])RX::_RX[p] -> interruptServiceRoutine();			\
}

#if defined(PCINT0_vect)
RX_ISR(0)
#endif
#if defined(PCINT1_vect)
RX_ISR(1)
#endif
#if defined(PCINT2_vect)
RX_ISR(2)
#endif
#if defined(PCINT3_vect)
RX_ISR(3)
#endif

