/******************************************************************************
 * Quadcopter-Library-v1
 * RX.cpp
 *  
 * This file contains functions for the receiver (RX). The signals from the
 * transmitter are measured based on pin change interrupts.
 *
 * TODO::calibration functions.
 * TODO::channel 5 functionalities.
 *
 * @author Rob Mertens
 * @version 1.1.1 14/08/2016
 ******************************************************************************/

#include "RX.h"

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer.
 ******************************************************************************/
RX::RX(volatile uint8_t * pcmskx, uint8_t pcint, t_alias alias, uint16_t periodMicrosecond, uint16_t maxMicrosecond, uint16_t minMicrosecond)
{	
	_pcmskx = pcmskx;							// Pass trough the Pin Change Mask Register to local variable.
	if(_pcmskx==PCMSK0)
	{	
		_pcie=0x01;
		_pin=(volatile uint8_t *)0x03;
	}
	else if(_pcmskx==PCMSK1)
	{
		_pcie=0x02;
		//TODO::PCINT8::PE0 -> 0x0C
		_pin=(volatile uint8_t *)0x103;
	}	
	else if(_pcmskx==PCMSK2)
	{	
		_pcie=0x04;
		_pin=(volatile uint8_t *)0x106;
	}
	else
	{;;}
	
	*_pcmskx |= pcint;							// Which Pin Change Interrupt Pins are used.
	
	for (uint8_t mask=0x01; mask<=0x80; mask<<=1)				// Loop for determining and splitting the Pin Change Interrupt Pins in different bytes.
    	{
    		int n = 0;
    		if(pcint & mask)
    		{
    			n++;
    			if(n==1)_ch1 |= mask;
    			if(n==2)_ch2 |= mask;
    			if(n==3)_ch3 |= mask;
    			if(n==4)_ch4 |= mask;
    		} 
	}
	
	_t	= timer8(alias);						// TIMER2.
	
	_periodMicroseconds = periodMicrosecond;
	_maxRxCycle = maxMicrosecond/periodMicrosecond;
	_minRxCycle = minMicrosecond/periodMicrosecond;
	
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
void RX::initialize(rx_mode mode)
{	
	_t.initialize(t_mode::NORMAL, t_interrupt::OVF);			// Set up the 8-bit timer prescaler value 64.
	_t.setPrescaler(1); 							// Maximum possible time for one timer run can be calculated.
	_t.reset();								// (timer_max) = (2^8 - 1) * (prescale / 16M) = 15,9375us
										// Per overflow flag the overflow word stores one timer value.
	enable();								// Maximum possible time with overflows can be calculated.
										// (ovf_max) = (2^32) * 15,9375us = 68451s = 19h00m51s
	setMode(mode);								// The drone will never fly 19 hours, so OK.
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
int8_t RX::setMode(rx_mode mode)
{
	int8_t ret = 0;
	_mode = rx_mode::NONE;
	
	switch(mode)
	{
		case rx_mode::M1:
			setMode2M1();
		
		case rx_mode::M2:
			setMode2M2();
		
		case rx_mode::NONE:
		default:
			ret = -1;
			return ret;
	}
	
	_mode = mode;
	
	return ret;
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
void RX::setMode2M1(void)
{
	assignThrottleChannel(0x01);
	assignRollChannel(0x04);
	assignPitchChannel(0x03);
	assignYawChannel(0x02);
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
void RX::setMode2M2(void)
{
	assignThrottleChannel(0x03);
	assignRollChannel(0x02);
	assignPitchChannel(0x01);
	assignYawChannel(0x04);
}


/*******************************************************************************
 * Method for assigning the throttle channel.
 * 
 * @param:
 * @return:
 ******************************************************************************/
int8_t RX::assignThrottleChannel(uint8_t channel)
{
	int8_t ret = 0;
	
	switch(channel)
	{
		case 0x01:
			_throttleChannel = &_channel1;
		
		case 0x02:
			_throttleChannel = &_channel2;
		
		case 0x03:
			_throttleChannel = &_channel3;
		
		case 0x04:
			_throttleChannel = &_channel4;
		
		default:
			ret = -1;
	}
	
	return ret;
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
float RX::getThrottleChannel()
{
	float rx = 0.0f;
	float dc = 0.0f;
	
	rx = *_throttleChannel/_periodMicroseconds;
	dc = rxc2dc(rx);
	
	return dc;
}

/*******************************************************************************
 * Method for assigning the throttle channel.
 * 
 * @param:
 * @return:
 ******************************************************************************/
int8_t RX::assignPitchChannel(uint8_t channel)
{
	int8_t ret = 0;
	
	switch(channel)
	{
		case 0x01:
		_pitchChannel = &_channel1;
		
		case 0x02:
		_pitchChannel = &_channel2;
		
		case 0x03:
		_pitchChannel = &_channel3;
		
		case 0x04:
		_pitchChannel = &_channel4;
		
		default:
		ret = -1;
	}
	
	return ret;
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
float RX::getPitchChannel()
{
	float rx = 0.0f;
	float dc = 0.0f;
	
	rx = *_pitchChannel/_periodMicroseconds;
	dc = rxc2dc(rx);
	
	return dc;

}

/*******************************************************************************
 * Method for assigning the throttle channel.
 * 
 * @param:
 * @return:
 ******************************************************************************/
int8_t RX::assignRollChannel(uint8_t channel)
{
	int8_t ret = 0;
	
	switch(channel)
	{
		case 0x01:
		_rollChannel = &_channel1;
		
		case 0x02:
		_rollChannel = &_channel2;
		
		case 0x03:
		_rollChannel = &_channel3;
		
		case 0x04:
		_rollChannel = &_channel4;
		
		default:
		ret = -1;
	}
	
	return ret;
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
float RX::getRollChannel()
{
	float rx = 0.0f;
	float dc = 0.0f;
	
	rx = *_rollChannel/_periodMicroseconds;
	dc = rxc2dc(rx);
	
	return dc;

}

/*******************************************************************************
 * Method for assigning the throttle channel.
 * 
 * @param:
 * @return:
 ******************************************************************************/
int8_t RX::assignYawChannel(uint8_t channel)
{
	int8_t ret = 0;
	
	switch(channel)
	{
		case 0x01:
		_yawChannel = &_channel1;
		
		case 0x02:
		_yawChannel = &_channel2;
		
		case 0x03:
		_yawChannel = &_channel3;
		
		case 0x04:
		_yawChannel = &_channel4;
		
		default:
		ret = -1;
	}
	
	return ret;
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
float RX::getYawChannel()
{
	float rx = 0.0f;
	float dc = 0.0f;
	
	rx = *_yawChannel/_periodMicroseconds;
	dc = rxc2dc(rx);
	
	return dc;

}

/*******************************************************************************
 * Method for mapping RX pulses 2 duty cycle.
 ******************************************************************************/
float RX::rxc2dc(float rxCycle)
{
	float dc;
	
	if(rxCycle >= _maxRxCycle)dc=1.0f;
	else if(rxCycle <= _minRxCycle)dc=0.0f;
	else{dc = ((1)/(_maxRxCycle - _minRxCycle))*rxCycle;}
	
	return dc;
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
uint8_t RX::getExtraChannel()
{
	//TODO::
	
	return 0x00;
}

/*******************************************************************************
 * ISR for the receiver class.
 ******************************************************************************/
void RX::interruptServiceRoutine(void)
{	
	//Channel 1.
	if(!(_lastChannel & _ch1) and *_pin & _ch1)
	{
		_channel1 = _t.getNonResetCount();
		_lastChannel |= _ch1;
	}
	else if(_lastChannel & _ch1 and !(*_pin & _ch1))
	{
		_channel1 -= _t.getNonResetCount();
		_lastChannel &= (_ch1 ^ 0xFF);
	}
	
	//Channel 2.
	if(!(_lastChannel & _ch2) and *_pin & _ch2)
	{
		_channel2  = _t.getNonResetCount();
		_lastChannel |= _ch2;
	}
	else if(_lastChannel & _ch2 and !(*_pin & _ch2))
	{
		_channel2 -= _t.getNonResetCount();
		_lastChannel &= (_ch2 ^ 0xFF);
	}
	
	//Channel 3.
	if(!(_lastChannel & _ch3) and *_pin & _ch3)
	{
		_channel3  = _t.getNonResetCount();
		_lastChannel |= _ch3;
	}
	else if(_lastChannel & _ch3 and !(*_pin & _ch3))
	{
		_channel3 -= _t.getNonResetCount();
		_lastChannel &= (_ch3 ^ 0xFF);
	}
	
	//Channel 4.
	if(!(_lastChannel & _ch4) and *_pin & _ch4)
	{
		_channel4  = _t.getNonResetCount();
		_lastChannel |= _ch4;
	}
	else if(_lastChannel & _ch4 and !(*_pin & _ch4))
	{	_channel4 -= _t.getNonResetCount();
		_lastChannel &= (_ch4 ^ 0xFF);
	}
}

/*******************************************************************************
 * Enable pinchange interrupts.
 ******************************************************************************/
void RX::enable(void)
{
	PCICR |= _pcie;
	cli();
}

/*******************************************************************************
 * Disable pinchange interrupts.
 ******************************************************************************/
void RX::disable(void)
{
	PCICR &= 0xF8;
}

/*******************************************************************************
 * Clear pinchange interrupts.
 ******************************************************************************/
void RX::clear(void)
{
	//*_pcmskx &= 0xFF
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

