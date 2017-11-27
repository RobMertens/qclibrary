/******************************************************************************
 * Quadcopter-Library-v1
 * ESC.cpp
 *
 * This file contains predefined functions for the ESC-class. A motor is
 * controlled by "asynchronous" Pulse Width Modulation (PWM). Asynchronous
 * points to the fact that the dutycycle can be set anywhere in the main
 * program. The motors are primarily controlled w/ hardware functions.
 *
 * Next sketch describes the quadcopter in top and bottom view. This sketch
 * describes the correct numbering and direction of rotation of each motor.
 *
 *	       TOP-VIEW			     		BOTTOM-VIEW
 *
 * 	/ 2 \   front   / 1 \		/ 2 \   front   / 1 \
 * 	\cw /           \ccw/		\ccw/           \cw /
 * 	   |::.........::|   		   |::.........::|
 * 	 l   |:       :|   r		 l   |:       :|   r
 * 	 e    |:     :|    i		 e    |:     :|    i
 * 	 f     |:::::|     g		 f     |:::::|     g
 * 	 t    |:     :|    h		 t    |:     :|    h
 * 	     |:       :|   t		     |:       :|   t
 * 	   |::.........::|		   |::.........::|
 * 	/ 3 \           / 4 \		/ 3 \           / 4 \
 * 	\ccw/   rear    \cw /		\cw /   rear    \ccw/

 * TODO::variable frequency and clock speed.
 *
 * @author: 	Rob Mertens
 * @date:			07/05/2017
 * @version: 	1.1.1
 ******************************************************************************/

#include "hardware/ESC.hpp"

namespace qc
{

namespace component
{

/** Constructors/destructors/overloading **************************************/
ESC::ESC(const avr::Timer16::Ptr& tptr, const avr::t_channel& channel,
	const uint16_t periodMicrosecond, const uint16_t maxMicrosecond,
	const uint16_t minMicrosecond)
{
	// ESC Timer.
	tptr_ = tptr;

	// IO.
	assign(channel);

	maxEscCycle_ = (double)(maxMicrosecond/periodMicrosecond);		// Is not 1.0f!
	minEscCycle_ = (double)(minMicrosecond/periodMicrosecond);		// Is not 0.0f!
}

ESC::ESC(const ESC& other)
{
	if(this != &other)
  {
		//TODO::factory pattern with copy-constructor.
		/*avr::Timer::Ptr tptr(new avr::Timer16(*other.tptr_));
		delete tptr_;
		tptr_ = tptr;*/

		maxEscCycle_ = other.maxEscCycle_;
		minEscCycle_ = other.minEscCycle_;
	}
}

ESC::~ESC(void)
{
	delete tptr_;
}

ESC& ESC::operator=(const ESC& other)
{
	if(this != &other)
  {
		//TODO::factory pattern with copy-constructor.
		/*avr::Timer::Ptr tptr(new avr::Timer16(*other.tptr_));
		delete tptr_;
		tptr_ = tptr;*/

		maxEscCycle_ = other.maxEscCycle_;
		minEscCycle_ = other.minEscCycle_;
	}
	return (*this);
}

/** ESC setting functions *****************************************************/
void ESC::arm(const uint16_t prescale,
							const uint16_t top)
{
	// TIMER16 Init PWM-mode.
	// t_tck = (1 * prescale) / 16M = 0,0625 µs.
	// t_max = 6,3e^(-8) * (2^16 - 1) = 4096 µs.
	// t_ocr = (4000 µs / 0,0625 µs) - 1 = 63999 (0xF9FF) ticks.
	tptr_->initialize(avr::t_mode::PWM_F, avr::t_channel::BC_TOP,
		avr::t_inverted::NORMAL);
	tptr_->setPrescaler(prescale);

	tptr_->setCompareValueA(top);

	writeMinSpeed();

	tptr_->reset();
}

void ESC::unarm(void)
{
	writeMinSpeed();
}

/** ESC runtime functions *****************************************************/
int8_t ESC::writeSpeed(const double dc)
{
	int8_t ret = 0;

	double escc = dc2Escc(dc);
	ret = (tptr_->*setDutyCycle)(escc);

	return ret;
}

int8_t ESC::writeMaxSpeed(void)
{
	int8_t ret = 0;

	ret = writeSpeed(dc2Escc(maxEscCycle_));

	return ret;
}

int8_t ESC::writeMinSpeed(void)
{
	int8_t ret = 0;

	ret = writeSpeed(dc2Escc(minEscCycle_));

	return ret;
}

/** Helper functions **********************************************************/
int8_t ESC::assign(const avr::t_channel& channel)
{
	int8_t ret = 0;

	if(channel==avr::t_channel::B)setDutyCycle = &avr::Timer16::setDutyCycleB;
	else if(channel==avr::t_channel::C)setDutyCycle = &avr::Timer16::setDutyCycleC;
	else{ret=-1;}

	return ret;
}

double ESC::dc2Escc(const double dutyCycle)
{
	double esc;

	if(dutyCycle >= 1.0f)esc=maxEscCycle_;
	else if(dutyCycle <= 0.0f)esc=minEscCycle_;
	else{esc = (maxEscCycle_ - minEscCycle_)*dutyCycle + minEscCycle_;}

	return esc;
}

}; //End namespace compoment.

}; //End namespace qc.
