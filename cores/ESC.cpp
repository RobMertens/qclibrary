/******************************************************************************
 * Quadcopter-Library-v1
 * ESCS.cpp
 *  
 * This file contains predefined functions for the ESC-class. The motors are
 * controlled by asynchronous Pulse Width Modulation (PWM). Next sketch
 * describes the quadcopter in bottom view.
 * 
 * 	/ 2 \   front   / 1 \
 * 	\ccw/           \cw /
 * 	   |::.........::|
 * 	 l   |:       :|   r
 * 	 e    |:     :|    i
 * 	 f     |:::::|     g
 * 	 t    |:     :|    h
 * 	     |:       :|   t
 * 	   |::.........::|
 * 	/ 3 \           / 4 \
 * 	\cw /   rear    \ccw/
 *
 * TODO::variable frequency and clock speed.
 *
 * @author: 	Rob Mertens
 * @date:	07/05/2017
 * @version: 	1.1.1
 ******************************************************************************/

#include <ESC.h>

/*******************************************************************************
 * Constructor for the ESCS-class.
 * 
 * By making an object with this constructor all local variables are set together
 * with the avr-timer (DEFAULT=TIMER1). This constructor is compatible for most
 * ESC's which have a DEFAULT microsecond PMW-range of minimum 1000us (motors off)
 * and maximum 2000us (motors at maximum throttle) or user specified.
 * 
 * @param: *ddr The Data Direction Register.
 * @param: ddrmsk The Data Direction Register mask.
 * @param: period The period length 			 [µs] (DEFAULT=4000).
 * @param: maxMicrosecond The full throttle pulse length [µs] (DEFAULT=2000).
 * @param: minMicrosecond The zero throttle pulse length [µs] (DEFAULT=1000).
 ******************************************************************************/
ESC::ESC(t_alias alias, t_channel channel, uint16_t periodMicrosecond, uint16_t maxMicrosecond, uint16_t minMicrosecond)
{	
	// ESC Timer.
	*_t 	= timer16(alias);						// TIMER16.
	
	// IO.
	assign(channel);						
	
	_maxEscCycle = (double)(maxMicrosecond/periodMicrosecond);		// Is not 1.0f!
	_minEscCycle = (double)(minMicrosecond/periodMicrosecond);		// Is not 0.0f!
}

/*******************************************************************************
 * Method for assigning a timer and PWM channel to an ESC. This should match
 * your hardware setup.
 * 
 * @param: alias The 16-bit timer alias.
 * @param: channel The 16-bit timer channel B or C. 
 ******************************************************************************/
int8_t ESC::assign(t_channel channel)
{
	int8_t ret = 0;
	
	if(channel==t_channel::B)setDutyCycle = &_t->setDutyCycleB;
	else if(channel==t_channel::C)setDutyCycle = &_t->setDutyCycleC;
	else{ret=-1}
	
	return ret;
}

/*******************************************************************************
 * Method for initializing the timer-settings.
 * The default is TIMER1 with no prescaler and timer overflow interrupt.
 *
 * @param: prescale The prescale mask value (DEFAULT=0x01).
 ******************************************************************************/
void ESC::arm(uint8_t prescale, uint16_t top)
{
	_t->initialize(t_mode::PWM_F, t_channel::BC_TOP, t_inverted::NORMAL);	// TIMER1 Init PWM-mode.
	_t->setPrescaler(prescale);						// t_tck = (1 * prescale) / 16M = 0,0625 µs.
										// t_max = 6,3e^(-8) * (2^16 - 1) = 4096 µs.
	_t->setCompareValueA(top);						// t_ocr = (4000 µs / 0,0625 µs) - 1 = 63999 (0xF9FF) ticks.
 	
	writeMinSpeed();
	
	_t->reset();
}

/*******************************************************************************
 * Method for writing variable PWM-pulses in length to the ESC's.
 *	
 * @param: dc1 The duty cycle for ESC1 in microseconds.
 ******************************************************************************/
void ESC::writeSpeed(float dc)
{
	*setDutyCycle(dc2Escc(dc));
}

/*******************************************************************************
 * Method for writing HIGH PWM-pulses to the ESC's.
 ******************************************************************************/
void ESC::writeMaxSpeed()
{
	writeSpeed(dc2Escc(_maxEscCycle));
}

/*******************************************************************************
 * Method for writing LOW PWM-pulses to the ESC's.
 ******************************************************************************/
void ESC::writeMinSpeed()
{
	writeSpeed(dc2Escc(_minEscCycle));
}

/*******************************************************************************
 * Method for writing LOW PWM-pulses to the ESC's.
 ******************************************************************************/
float ESC::dc2Escc(float dutyCycle)
{
	float esc;
	
	if(dutyCycle >= 1.0f)esc=_maxEscCycle;
	else if(dutyCycle <= 0.0f)esc=_minEscCycle;
	else{esc = (_maxEscCycle - _minEscCycle)*dutyCycle + _minEscCycle;}
	
	return esc;
}
