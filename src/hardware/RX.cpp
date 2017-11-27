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
 * @author 	Rob Mertens
 * @date 		14/08/2016
 * @version 1.1.1
 ******************************************************************************/

#include "hardware/RX.hpp"

namespace qc
{

namespace component
{

/** Constructors/destructors/overloads ****************************************/
RX::RX(const volatile uint8_t * const& pcmskx, const uint8_t pcint,
	const avr::Timer::Ptr& tptr, const uint16_t periodMicrosecond,
	const uint16_t maxMicrosecond, const uint16_t minMicrosecond)
{
	// Pass trough the Pin Change Mask Register to local variable.
	pcmskx_ = pcmskx;
	if(pcmskx_==PCMSK0)
	{
		pcie_=0x01;
		pin_=(volatile uint8_t *)0x03;
	}
	else if(pcmskx_==PCMSK1)
	{
		pcie_=0x02;
		//TODO::PCINT8::PE0 -> 0x0C
		pin_=(volatile uint8_t *)0x103;
	}
	else if(pcmskx_==PCMSK2)
	{
		pcie_=0x04;
		pin_=(volatile uint8_t *)0x106;
	}
	else
	{;;}

	// Which Pin Change Interrupt Pins are used.
	*pcmskx_ |= pcint;

	// Loop for determining and splitting the Pin Change Interrupt Pins in different bytes.
	for (uint8_t mask=0x01; mask<=0x80; mask<<=1)
    	{
    		int n = 0;
    		if(pcint & mask)
    		{
    			n++;
    			if(n==1)ch1_ |= mask;
    			if(n==2)ch2_ |= mask;
    			if(n==3)ch3_ |= mask;
    			if(n==4)ch4_ |= mask;
    		}
	}

	tptr_	= tptr;

	periodMicroseconds_ = periodMicrosecond;
	maxRxCycle_ = maxMicrosecond/periodMicrosecond;
	minRxCycle_ = minMicrosecond/periodMicrosecond;

}

RX::~RX(void)
{
	//Registers.
	delete pcmskx_;
	delete pin_;

	//Channels mapping.
	delete channelT_;
	delete channelR_;
	delete channelP_;
	delete channelY_;
	delete channelE_;

	//Timer pointer.
	delete tptr_;

	//Interrupt instances.
	//Delete the static instance that corresponts with this RX.
	//Delete this instance from static list.
	for(size_t n = 0; n < sizeof(__RX__); ++n)
	{
		//Do both pointers point to the same adress?
		//If so, delete pointer in the static list.
		if(__RX__[n]==this)delete __RX__[n];
	}
}

RX& RX::operator=(const RX& other)
{
	if(this != &other)
  {
		//Volatiles.
		uint8_t *pcmskx(new volatile uint8_t(*other.pcmskx_));
		delete pcmskx_;
		pcmskx_ = pcmskx;
		uint8_t *pin(new volatile uint8_t(*other.pin_));
		delete pin_;
		pin_ = pin;

		//Registers.
		pcie_ = other.pcie_;

		//Channel locations.
		ch1_ = other.ch1_;
		ch2_ = other.ch2_;
		ch3_ = other.ch3_;
		ch4_ = other.ch4_;
		ch5_ = other.ch5_;
		lastChannel_ = other.lastChannel_;

		//Channel variables.
		channel1_ = other.channel1_;
		channel2_ = other.channel2_;
		channel3_ = other.channel3_;
		channel4_ = other.channel4_;
		channel5_ = other.channel5_;
		offset1_ = other.offset1_;
		offset2_ = other.offset2_;
		offset3_ = other.offset3_;
		offset4_ = other.offset4_;
		offset5_ = other.offset5_;

		//TODO::factory pattern with copy-constructor.
		//For now, point to the existing instance.
		avr::Timer::Ptr tptr = &(*other.tptr_);
		delete tptr_;
		tptr_ = tptr;

		//RX mode.
		mode_ = other.mode_;

		//Channels mapping.
		uint32_t *channelT(new uint32_t(*other.channelT_));
		delete channelT_;
		channelT_ = channelT;
		uint32_t *channelR(new uint32_t(*other.channelR_));
		delete channelR_;
		channelR_ = channelR;
		uint32_t *channelP(new uint32_t(*other.channelP_));
		delete channelP_;
		channelP_ = channelP;
		uint32_t *channelY(new uint32_t(*other.channelY_));
		delete channelY_;
		channelY_ = channelY;
		uint32_t *channelE(new uint32_t(*other.channelE_));
		delete channelE_;
		channelE_ = channelE;

  }
	return (*this);
}

/** RX settings functions ***********************************************/
void RX::initialize(const Mode& mode)
{
	// Set up the 8-bit timer prescaler value 64.
	// Maximum possible time for one timer run can be calculated.
	// (timer_max) = (2^8 - 1) * (prescale / 16M) = 15,9375us
	// Per overflow flag the overflow word stores one timer value.
	// Maximum possible time with overflows can be calculated.
	// (ovf_max) = (2^32) * 15,9375us = 68451s = 19h00m51s
	// The drone will never fly 19 hours, so OK.
	tptr_->initialize(avr::t_mode::NORMAL,
		avr::t_interrupt::OVF);
	tptr_->setPrescaler(1);
	tptr_->reset();

	//Enable interrupts.
	enable();

	//Set the receiver mode.
	setMode(mode);
}

int8_t RX::setMode(const Mode& mode)
{
	int8_t ret = 0;
	mode_ = Mode::NONE;

	//Assign all channels.
	//TODO::extra channel.
	switch(mode)
	{
		case Mode::M1 :
			ret = assignThrottleChannel(0x01);
			ret += assignRollChannel(0x04);
			ret += assignPitchChannel(0x03);
			ret += assignYawChannel(0x02);
			break;
		case Mode::M2 :
			ret = assignThrottleChannel(0x03);
			ret += assignRollChannel(0x02);
			ret += assignPitchChannel(0x01);
			ret += assignYawChannel(0x04);
			break;
		case Mode::NONE :
		default :
			ret = -1;
			return ret;
	}

	//Check return statement.
	if(ret==0)mode_ = mode;

	return ret;
}

int8_t RX::assignThrottleChannel(const uint8_t channel)
{
	int8_t ret = 0;

	switch(channel)
	{
		case 0x01:
			channelT_ = &channel1_;
			break;
		case 0x02:
			channelT_ = &channel2_;
			break;
		case 0x03:
			channelT_ = &channel3_;
			break;
		case 0x04:
			channelT_ = &channel4_;
			break;
		default:
			ret = -1;
			break;
	}

	return ret;
}

int8_t RX::assignPitchChannel(const uint8_t channel)
{
	int8_t ret = 0;

	switch(channel)
	{
		case 0x01:
			channelP_ = &channel1_;
			break;
		case 0x02:
			channelP_ = &channel2_;
			break;
		case 0x03:
			channelP_ = &channel3_;
			break;
		case 0x04:
			channelP_ = &channel4_;
			break;
		default:
			ret = -1;
			break;
	}

	return ret;
}

int8_t RX::assignRollChannel(const uint8_t channel)
{
	int8_t ret = 0;

	switch(channel)
	{
		case 0x01:
			channelR_ = &channel1_;
			break;
		case 0x02:
			channelR_ = &channel2_;
			break;
		case 0x03:
			channelR_ = &channel3_;
			break;
		case 0x04:
			channelR_ = &channel4_;
			break;
		default:
			ret = -1;
			break;
	}

	return ret;
}

int8_t RX::assignYawChannel(const uint8_t channel)
{
	int8_t ret = 0;

	switch(channel)
	{
		case 0x01:
			channelY_ = &channel1_;
			break;
		case 0x02:
			channelY_ = &channel2_;
			break;
		case 0x03:
			channelY_ = &channel3_;
			break;
		case 0x04:
			channelY_ = &channel4_;
			break;
		default:
			ret = -1;
			break;
	}

	return ret;
}

int8_t RX::assignExtraChannel(void)
{
	channelE_ = &channel5_;
}

/** RX runtime functions ************************************************/
uint8_t RX::getThrottleChannel(void)
{
	//TODO::return the channel.
	return 0x00;
}

double RX::getThrottleInput(void)
{
	double rx = 0.0f;
	double dc = 0.0f;

	rx = *channelT_/periodMicroseconds_;
	dc = rxc2dc(rx, 0.8f);							//Preserve some top-margin.

	return dc;
}

uint8_t RX::getPitchChannel(void)
{
	//TODO::return the channel.
	return 0x00;
}

double RX::getPitchInput(void)
{
	double rx = 0.0f;
	double dc = 0.0f;

	rx = *channelP_/periodMicroseconds_;
	dc = rxc2dc(rx, 1.0f);

	return dc;
}

uint8_t RX::getRollChannel(void)
{
	//TODO::return the channel.
	return 0x00;
}

double RX::getRollInput(void)
{
	double rx = 0.0f;
	double dc = 0.0f;

	rx = *channelR_/periodMicroseconds_;
	dc = rxc2dc(rx, 1.0f);

	return dc;
}

uint8_t RX::getYawChannel(void)
{
	//TODO::return the channel.
	return 0x00;
}

double RX::getYawInput(void)
{
	double rx = 0.0f;
	double dc = 0.0f;

	rx = *channelY_/periodMicroseconds_;
	dc = rxc2dc(rx, 1.0f);

	return dc;
}

uint8_t RX::getExtraChannel(void)
{
	return 0x05;
}

double RX::getExtraInput(void)
{
	//TODO::
}

/** Helper functions **********************************************************/
double RX::rxc2dc(const double rxCycle, const double maxDc)
{
	double dc;

	if(rxCycle >= maxRxCycle_)dc = maxDc;
	else if(rxCycle <= minRxCycle_)dc = 0.0f;
	else{dc = ((1)/(maxRxCycle_ - minRxCycle_))*rxCycle;}

	return dc;
}

/** Interrupt functionality overrides *****************************************/
void RX::interruptServiceRoutine(void)
{
	//Channel 1.
	if(!(lastChannel_ & ch1_) and *pin_ & ch1_)
	{
		channel1_ = tptr_->getNonResetCount();
		lastChannel_ |= ch1_;
	}
	else if(lastChannel_ & ch1_ and !(*pin_ & ch1_))
	{
		channel1_ = tptr_->getNonResetCount() - (channel1_ + offset1_);
		lastChannel_ &= (ch1_ ^ 0xFF);
	}

	//Channel 2.
	if(!(lastChannel_ & ch2_) and *pin_ & ch2_)
	{
		channel2_  = tptr_->getNonResetCount();
		lastChannel_ |= ch2_;
	}
	else if(lastChannel_ & ch2_ and !(*pin_ & ch2_))
	{
		channel2_ = tptr_->getNonResetCount() - (channel2_ + offset2_);
		lastChannel_ &= (ch2_ ^ 0xFF);
	}

	//Channel 3.
	if(!(lastChannel_ & ch3_) and *pin_ & ch3_)
	{
		channel3_  = tptr_->getNonResetCount();
		lastChannel_ |= ch3_;
	}
	else if(lastChannel_ & ch3_ and !(*pin_ & ch3_))
	{
		channel3_ = tptr_->getNonResetCount() - (channel3_ + offset3_);
		lastChannel_ &= (ch3_ ^ 0xFF);
	}

	//Channel 4.
	if(!(lastChannel_ & ch4_) and *pin_ & ch4_)
	{
		channel4_  = tptr_->getNonResetCount();
		lastChannel_ |= ch4_;
	}
	else if(lastChannel_ & ch4_ and !(*pin_ & ch4_))
	{
		channel4_ = tptr_->getNonResetCount() - (channel4_ + offset4_);
		lastChannel_ &= (ch4_ ^ 0xFF);
	}

	//Channel 5.
	//TODO::

	//TODO::if no timer is saved reset the timer. Otherwise the timer will
	//overflow and jumps will occur. This is however not 100% failsafe.
}

void RX::enable(void)
{
	PCICR |= pcie_;
	cli();
}

void RX::disable(void)
{
	PCICR &= 0xF8;
	//sei();
}

void RX::clear(void)
{
	*pcmskx_ &= 0xFF;
	//TODO::look at PCIFR register.
}

/** Interrupt preprocessor functions ******************************************/
/**
 * @brief The private static list of receivers. Each pin change interrupt
 *				vector has it's own instance in the static list. If the corresponding
 *				interrupt vector is set, the instance is added to this list.
 *
 * We initiate the private function in de .source file which does has acces to
 * the static private member list.
 *
 * TODO::different avrs.
 */
RX::Ptr RX::__RX__[4] = {};

/**
 * @brief This preporcessor method creates an ISR mapping function for each
 *				available pin change interrupt vector. This is now done for the
 *				arduino MEGA2560 but in the future UNO will be supported.
 *
 * TODO::different avrs.
 */
#define RX_ISR(p)	ISR(PCINT ## p ## _vect)										\
{																															\
	if(RX::__RX__[p])RX::__RX__[p] -> interruptServiceRoutine();\
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

}; //End namespace component.

}; //End namespace qc.
