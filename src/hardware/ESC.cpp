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
 * @date:	07/05/2017
 * @version: 	1.1.1
 ******************************************************************************/

#include "hardware/ESC.h"

/*******************************************************************************
 * @brief Constructor for the ESC-class.
 * 				By making an object with this constructor all local variables are
 * 				set together with the avr-timer. This constructor is compatible for
 *				most ESC's which have a DEFAULT microsecond PMW-range of minimum
 *				1000us (motors off)	and maximum 2000us (motors at maximum throttle)
 * 				or user specified.
 * @param: alias
 * @param: channel
 * @param: periodMicrosecond The period length [µs] (DEFAULT=4000).
 * @param: maxMicrosecond The full throttle pulse length [µs] (DEFAULT=2000).
 * @param: minMicrosecond The zero throttle pulse length [µs] (DEFAULT=1000).
 ******************************************************************************/
ESC::ESC(const t_settings::alias& alias,
				 const t_settings::channel& channel,
				 const uint16_t periodMicrosecond,
				 const uint16_t maxMicrosecond,
				 const uint16_t minMicrosecond)
{
	// ESC Timer.
	_t = timer16(alias);							// TIMER16.

	// IO.
	assign(channel);

	_maxEscCycle = (double)(maxMicrosecond/periodMicrosecond);		// Is not 1.0f!
	_minEscCycle = (double)(minMicrosecond/periodMicrosecond);		// Is not 0.0f!
}

/*******************************************************************************
 * @brief Method for assigning a timer and PWM channel to an ESC. This should match
 * 				your hardware setup.
 * @param: alias The 16-bit timer alias.
 * @param: channel The 16-bit timer channel B or C.
 * @return: ret A return value ? 0 for successful : -1 for unsuccessful.
 ******************************************************************************/
int8_t ESC::assign(const t_settings::channel& channel)
{
	int8_t ret = 0;

	if(channel==t_settings::channel::B)setDutyCycle = &timer16::setDutyCycleB;
	else if(channel==t_settings::channel::C)setDutyCycle = &timer16::setDutyCycleC;
	else{ret=-1;}

	return ret;
}

/*******************************************************************************
 * @brief Method for initializing the timer-settings.
 * 				The default is TIMER1 with no prescaler and timer overflow interrupt.
 * @param: prescale The prescale mask value (DEFAULT=0x01).
 ******************************************************************************/
void ESC::arm(const uint16_t prescale,
							const uint16_t top)
{
	_t.initialize(t_settings::mode::PWM_F,
								t_settings::channel::BC_TOP,
								t_settings::inverted::NORMAL);																	// TIMER16 Init PWM-mode.
	_t.setPrescaler(prescale);																										// t_tck = (1 * prescale) / 16M = 0,0625 µs.
																																								// t_max = 6,3e^(-8) * (2^16 - 1) = 4096 µs.
	_t.setCompareValueA(top);																											// t_ocr = (4000 µs / 0,0625 µs) - 1 = 63999 (0xF9FF) ticks.

	writeMinSpeed();

	_t.reset();
}

/*******************************************************************************
 * @brief TODO::safely shut down the motors.
 ******************************************************************************/
void ESC::unarm(void)
{
	;;
}

/*******************************************************************************
 * @brief Method for writing variable PWM-pulses in length to the ESC's.
 * @param dc The duty cycle for the ESC in microseconds.
 * @return ret A return value for debugging ? 0 successful : -1 unsuccessful.
 ******************************************************************************/
int8_t ESC::writeSpeed(const float dc)
{
	int8_t ret = 0;
	
	float escc = dc2Escc(dc);
	ret = (_t.*setDutyCycle)(escc);

	return ret;
}

/*******************************************************************************
 * @brief Method for writing HIGH PWM-pulses to the ESC's.
 * @return ret A return value for debugging ? 0 successful : -1 unsuccessful.
 ******************************************************************************/
int8_t ESC::writeMaxSpeed(void)
{
	int8_t ret = 0;

	ret = writeSpeed(dc2Escc(_maxEscCycle));

	return ret;
}

/*******************************************************************************
 * @brief Method for writing LOW PWM-pulses to the ESC's.
 * @return ret A return value for debugging ? 0 successful : -1 unsuccessful.
 ******************************************************************************/
int8_t ESC::writeMinSpeed(void)
{
	int8_t ret = 0;

	ret = writeSpeed(dc2Escc(_minEscCycle));

	return ret;
}

/*******************************************************************************
 * @brief Method for writing LOW PWM-pulses to the ESC's.
 * @param
 * @return
 ******************************************************************************/
float ESC::dc2Escc(const float dutyCycle)
{
	float esc;

	if(dutyCycle >= 1.0f)esc=_maxEscCycle;
	else if(dutyCycle <= 0.0f)esc=_minEscCycle;
	else{esc = (_maxEscCycle - _minEscCycle)*dutyCycle + _minEscCycle;}

	return esc;
}
