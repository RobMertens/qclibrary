/******************************************************************************
 * Quadcopter-Library-v1
 * RX.cpp
 *
 * This file contains functions for the receiver (RX). The signals from the
 * transmitter are measured based on pin change interrupts.
 *
 * TODO::calibration functions.
 * TODO::channel 5 functionalities.
 * TODO::support radians/degrees -> sensitivity functionalities.
 *
 * @author Rob Mertens
 * @version 1.1.1 14/08/2016
 ******************************************************************************/

#include "hardware/RX.h"

/*******************************************************************************
 * Constructor for the ESC-class. By making an object with this constructor
 * all local variables are set together with the avr-timer.
 ******************************************************************************/
RX::RX(const volatile uint8_t * const& pcmskx,
			 const uint8_t pcint,
			 const t_settings::alias alias,
			 const uint16_t periodMicrosecond,
			 const uint16_t maxMicrosecond,
			 const uint16_t minMicrosecond)
{
	// Pass trough the Pin Change Mask Register to local variable.
	_pcmskx = pcmskx;
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

	// Which Pin Change Interrupt Pins are used.
	*_pcmskx |= pcint;

	// Loop for determining and splitting the Pin Change Interrupt Pins in different bytes.
	for (uint8_t mask=0x01; mask<=0x80; mask<<=1)
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
void RX::initialize(const rx_settings::mode mode)
{
	_t.initialize(t_mode::NORMAL, t_interrupt::OVF);			// Set up the 8-bit timer prescaler value 64.
	_t.setPrescaler(1); 																	// Maximum possible time for one timer run can be calculated.
	_t.reset();																						// (timer_max) = (2^8 - 1) * (prescale / 16M) = 15,9375us
																												// Per overflow flag the overflow word stores one timer value.
	enable();																							// Maximum possible time with overflows can be calculated.
																												// (ovf_max) = (2^32) * 15,9375us = 68451s = 19h00m51s
	setMode(mode);																				// The drone will never fly 19 hours, so OK.
}

/*******************************************************************************
 * Method for initializing the receiver.
 ******************************************************************************/
int8_t RX::setMode(const rx_settings::mode mode)
{
	int8_t ret = 0;
	_mode = rx_settings::mode::NONE;

	switch(mode)
	{
		case rx_settings::mode::M1 :
			_chT = 0x01;
			_chR = 0x04;
			_chP = 0x03;
			_chY = 0x02;
			break;
		case rx_settings::mode::M2 :
			_chT = 0x03;
			_chR = 0x02;
			_chP = 0x01;
			_chY = 0x04;
			break;
		case rx_settings::mode::NONE :
		default :
			ret = -1;
			return ret;
	}

	//Assign all channels.
	//TODO::extra channel.
	ret = assignThrottleChannel(_chT)
				+ assignRollChannel(_chR)
				+ assignPitchChannel(_chP)
				+ assignYawChannel(_chY);

	if(ret==0)_mode = mode;

	return ret;
}

/*******************************************************************************
 * @brief Method for assigning the throttle channel.
 * @param
 * @return
 ******************************************************************************/
int8_t RX::assignThrottleChannel(const uint8_t channel)
{
	int8_t ret = 0;

	switch(channel)
	{
		case 0x01:
			_channelT = &_channel1;
			break;
		case 0x02:
			_channelT = &_channel2;
			break;
		case 0x03:
			_channelT = &_channel3;
			break;
		case 0x04:
			_channelT = &_channel4;
			break;
		default:
			ret = -1;
			break;
	}

	return ret;
}

/*******************************************************************************
 * @brief Method for initializing the receiver.
 * @return The the throlle channel mask.
 ******************************************************************************/
uint8_t RX::getThrottleChannel(void)
{
	return _chT;
}

/*******************************************************************************
 * @brief Method for initializing the receiver.
 * @return dc The actual throttle value in dutycycle.
 ******************************************************************************/
float RX::getThrottleInput(void)
{
	float rx = 0.0f;
	float dc = 0.0f;

	rx = *_channelT/_periodMicroseconds;
	dc = rxc2dc(rx, 0.8f);							//Preserve some top-margin.

	return dc;
}

/*******************************************************************************
 * Method for assigning the throttle channel.
 * @param:
 * @return:
 ******************************************************************************/
int8_t RX::assignPitchChannel(const uint8_t channel)
{
	int8_t ret = 0;

	switch(channel)
	{
		case 0x01:
			_channelP = &_channel1;
			break;
		case 0x02:
			_channelP = &_channel2;
			break;
		case 0x03:
			_channelP = &_channel3;
			break;
		case 0x04:
			_channelP = &_channel4;
			break;
		default:
			ret = -1;
			break;
	}

	return ret;
}

/*******************************************************************************
 * @brief Method for initializing the receiver.
 * @return
 ******************************************************************************/
uint8_t RX::getPitchChannel(void)
{
	return _chP;
}

/*******************************************************************************
 * @brief Method for initializing the receiver.
 * @return
 ******************************************************************************/
float RX::getPitchInput(void)
{
	float rx = 0.0f;
	float dc = 0.0f;

	rx = *_channelP/_periodMicroseconds;
	dc = rxc2dc(rx, 1.0f);

	return dc;
}

/*******************************************************************************
 * Method for assigning the throttle channel.
 * @param
 * @return
 ******************************************************************************/
int8_t RX::assignRollChannel(const uint8_t channel)
{
	int8_t ret = 0;

	switch(channel)
	{
		case 0x01:
			_channelR = &_channel1;
			break;
		case 0x02:
			_channelR = &_channel2;
			break;
		case 0x03:
			_channelR = &_channel3;
			break;
		case 0x04:
			_channelR = &_channel4;
			break;
		default:
			ret = -1;
			break;
	}

	return ret;
}

/*******************************************************************************
 * @brief Method for initializing the receiver.
 *				TODO::introduce some deadband.
 * @return
 ******************************************************************************/
uint8_t RX::getRollChannel(void)
{
	return _chR;
}

/*******************************************************************************
 * @brief Method for initializing the receiver.
 *				TODO::introduce some deadband.
 * @return
 ******************************************************************************/
float RX::getRollInput(void)
{
	float rx = 0.0f;
	float dc = 0.0f;

	rx = *_channelR/_periodMicroseconds;
	dc = rxc2dc(rx, 1.0f);

	return dc;
}

/*******************************************************************************
 * @brief Method for assigning the throttle channel.
 * @param
 * @return
 ******************************************************************************/
int8_t RX::assignYawChannel(const uint8_t channel)
{
	int8_t ret = 0;

	switch(channel)
	{
		case 0x01:
			_channelY = &_channel1;
			break;
		case 0x02:
			_channelY = &_channel2;
			break;
		case 0x03:
			_channelY = &_channel3;
			break;
		case 0x04:
			_channelY = &_channel4;
			break;
		default:
			ret = -1;
			break;
	}

	return ret;
}

/*******************************************************************************
 * @brief Method for initializing the receiver.
 * @return
 ******************************************************************************/
uint8_t RX::getYawChannel(void)
{
	return _chY;
}

/*******************************************************************************
 * @brief Method for initializing the receiver.
 * @return
 ******************************************************************************/
float RX::getYawInput(void)
{
	float rx = 0.0f;
	float dc = 0.0f;

	rx = *_channelY/_periodMicroseconds;
	dc = rxc2dc(rx, 1.0f);

	return dc;
}

/*******************************************************************************
 * @brief Method for assigning the throttle channel.
 * @param
 * @return
 ******************************************************************************/
int8_t RX::assignExtraChannel(void)
{
	_channelE = &_channel5;
}

/*******************************************************************************
 * @brief Method for initializing the receiver.
 * @return
 ******************************************************************************/
uint8_t RX::getExtraChannel(void)
{
	return 0x05;
}

/*******************************************************************************
 * @brief Method for initializing the receiver.
 * @return
 ******************************************************************************/
float RX::getExtraInput(void)
{
	//TODO::
}

/*******************************************************************************
 * @brief Method for mapping RX pulses 2 duty cycle.
 * @return
 ******************************************************************************/
float RX::rxc2dc(const float rxCycle, const  float maxDc)
{
	float dc;

	if(rxCycle >= _maxRxCycle)dc = maxDc;
	else if(rxCycle <= _minRxCycle)dc = 0.0f;
	else{dc = ((1)/(_maxRxCycle - _minRxCycle))*rxCycle;}

	return dc;
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
		_channel1 = _t.getNonResetCount() - (_channel1 + _offset1);
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
		_channel2 = _t.getNonResetCount() - (_channel2 + _offset2);
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
		_channel3 = _t.getNonResetCount() - (_channel3 + _offset3);
		_lastChannel &= (_ch3 ^ 0xFF);
	}

	//Channel 4.
	if(!(_lastChannel & _ch4) and *_pin & _ch4)
	{
		_channel4  = _t.getNonResetCount();
		_lastChannel |= _ch4;
	}
	else if(_lastChannel & _ch4 and !(*_pin & _ch4))
	{
		_channel4 = _t.getNonResetCount() - (_channel4 + _offset4);
		_lastChannel &= (_ch4 ^ 0xFF);
	}

	//Channel 5.
	//TODO::
}

/*******************************************************************************
 * @brief Enable pinchange interrupts.
 ******************************************************************************/
void RX::enable(void)
{
	PCICR |= _pcie;
	cli();
}

/*******************************************************************************
 * @brief Disable pinchange interrupts.
 ******************************************************************************/
void RX::disable(void)
{
	PCICR &= 0xF8;
}

/*******************************************************************************
 * @brief Clear pinchange interrupts.
 ******************************************************************************/
void RX::clear(void)
{
	//*_pcmskx &= 0xFF
}

/*******************************************************************************
 * Global forward declaration.
 ******************************************************************************/
RX::ptr RX::_RX[4] = {};

/*******************************************************************************
 * _vector__x -> ISR().
 *
 * TODO::different avrs.
 *
 * Call from self.
 ******************************************************************************/
#define RX_ISR(p)																				\
ISR(PCINT ## p ## _vect)																\
{																												\
	if(RX::_RX[p])RX::_RX[p] -> interruptServiceRoutine();\
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
