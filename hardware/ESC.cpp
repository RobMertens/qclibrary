/******************************************************************************
 * Quadcopter-Library-v1
 * ESC.cpp
 *  
 * This file contains predefined functions for the ESC-class. The motors are
 * controlled by asynchronous Pulse Width Modulation (PWM).
 * 
 * TODO::add number of motors.
 * TODO::assign esc(X) -> motor(Y) functions.
 * TODO::variable frequency and clock speed.
 *
 * @author: 	Rob Mertens
 * @date:	14/08/2016
 * @version: 	1.1.1
 ******************************************************************************/

#include <ESC.h>

/*******************************************************************************
 * Constructor for the ESC-class.
 * 
 * By making an object with this constructor all local variables are set together
 * with the avr-timer (DEFAULT=TIMER1). This constructor is compatible for most
 * ESC's which have a DEFAULT microsecond PMW-range of minimum 1000us (motors off)
 * and maximum 2000us (motors at maximum throttle) or user specified.
 * 
 * @param: *ddr The Data Direction Register.
 * @param: ddrmsk The Data Direction Register mask.
 * @param: period The period length [µs] (DEFAULT=4000).
 * @param: maxMicrosecond The full throttle pulse length [µs] (DEFAULT=2000).
 * @param: minMicrosecond The zero throttle pulse length [µs] (DEFAULT=1000).
 ******************************************************************************/
ESC::ESC(volatile uint8_t * ddr, uint8_t ddrmsk, uint16_t periodMicrosecond, uint16_t maxMicrosecond, uint16_t minMicrosecond)
{	
	// IO.
	_ddr  	= ddr;								// Put definitions in local variables.
	*_ddr 	|= ddrmsk;
	
	// ESC Timer.
	_t1 	= timer16(t_alias::T1);						// TIMER1.
	_t3	= timer16(t_alias::T3);						// TIMER3.

	_maxEscCycle = (double)(maxMicrosecond/periodMicrosecond);		// Is not 1.0f!
	_minEscCycle = (double)(minMicrosecond/periodMicrosecond);		// Is not 0.0f!
}

/*******************************************************************************
 * Method for initializing the timer-settings.
 * The default is TIMER1 with no prescaler and timer overflow interrupt.
 *
 * @param: prescale The prescale mask value (DEFAULT=0x01).
 ******************************************************************************/
void ESC::arm(uint8_t prescale=0x01)
{
	_t1.initialize(t_mode::F_PWM, t_channel::ABC, bool inverted=false);	// TIMER1 Init PWM-mode.
	_t1.setPrescaler(prescale);						// t_tck = (1 * prescale) / 16M = 0,0625 µs.
										// t_max = 6,3e^(-8) * (2^16 - 1) = 4096 µs.
	_t1.setCompareValueA(0xF9FF);						// t_ocr = (4000 µs / 0,0625 µs) - 1 = 63999 (0xF9FF) ticks.
 	_t1.setDutyCycleB(_minEscCycle);
	_t1.setDutyCycleC(_minEscCycle);
	
	_t3.initialize(t_mode::F_PWM, t_channel::ABC, bool inverted=false);	// TIMER3 same.
	_t3.setPrescaler(prescale);
	_t3.setCompareValueA(0xF9FF);
	_t3.setDutyCycleB(_minEscCycle);
	_t3.setDutyCycleC(_minEscCycle);
	
	_t1.reset();								// Start PWM.
	_t3.reset();
}

/*******************************************************************************
 * Method for writing variable PWM-pulses in length to the ESC's.
 *
 * TODO::use interrupts to do other things.
 *	
 * @param: dc1 The duty cycle for ESC1 in microseconds.
 * @param: dc2 The duty cycle for ESC2 in microseconds.
 * @param: dc3 The duty cycle for ESC3 in microseconds.
 * @param: dc4 The duty cycle for ESC4 in microseconds.
 ******************************************************************************/
void ESC::writeSpeed(float dc1, float dc2, float dc3, float dc4)
{
	_t1.setDutyCycleB(dc2Escc(dc1));			// Clockwise.
	_t1.setDutyCycleC(dc2Escc(dc2));
	
	_t3.setDutyCycleB(dc2Escc(dc3));			// Counter Clockwise.
	_t3.setDutyCycleC(dc2Escc(dc4));
}

/*******************************************************************************
 * Method for writing HIGH PWM-pulses to the ESC's.
 ******************************************************************************/
void ESC::writeMaxSpeed()
{
	_t1.setDutyCycleB(_maxEscCycle);					// Clockwise.
	_t1.setDutyCycleC(_maxEscCycle);
	
	_t3.setDutyCycleB(_maxEscCycle);					// Counter Clockwise.
	_t3.setDutyCycleC(_maxEscCycle);
}

/*******************************************************************************
 * Method for writing LOW PWM-pulses to the ESC's.
 ******************************************************************************/
void ESC::writeMinSpeed()
{
	_t1.setDutyCycleB(_minEscCycle);					// Clockwise.
	_t1.setDutyCycleC(_minEscCycle);
	
	_t3.setDutyCycleB(_minEscCycle);					// Counter Clockwise.
	_t3.setDutyCycleC(_minEscCycle);
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
