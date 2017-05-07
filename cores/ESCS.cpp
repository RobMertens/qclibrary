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
 * TODO::add number of motors.
 *
 * @author: 	Rob Mertens
 * @date:	14/08/2016
 * @version: 	1.1.1
 ******************************************************************************/

#include <ESCS.h>

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
ESCS::ESCS(t_alias ax, t_alias ay, uint16_t periodMicrosecond, uint16_t maxMicrosecond, uint16_t minMicrosecond)
{	
	// ESC Timer.
	*_tx 	= timer16(ax);							// TIMERX.
	*_ty	= timer16(ay);							// TIMERY.
	
	// IO.
	_ddrx	= _tx->getDataDirectionRegister();				// Put definitions in local variables.
	*_ddrx 	|= ddrxmsk;							//TODO::set ddrmask channel B and C as output.
	
	_ddry	= _ty->getDataDirectionRegister();				// Put definitions in local variables.
	*_ddry 	|= ddrymsk;							//TODO::set ddrmask channel B and C as output.
	
	_maxEscCycle = (double)(maxMicrosecond/periodMicrosecond);		// Is not 1.0f!
	_minEscCycle = (double)(minMicrosecond/periodMicrosecond);		// Is not 0.0f!
}

/*******************************************************************************
 * Method for initializing the timer-settings.
 * The default is TIMER1 with no prescaler and timer overflow interrupt.
 *
 * @param: prescale The prescale mask value (DEFAULT=0x01).
 ******************************************************************************/
void ESCS::arm(uint8_t prescale, uint16_t top)
{
	_tx->initialize(t_mode::PWM_F, t_channel::BC_TOP, t_inverted::NORMAL);	// TIMER1 Init PWM-mode.
	_tx->setPrescaler(prescale);						// t_tck = (1 * prescale) / 16M = 0,0625 µs.
										// t_max = 6,3e^(-8) * (2^16 - 1) = 4096 µs.
	_tx->setCompareValueA(top);						// t_ocr = (4000 µs / 0,0625 µs) - 1 = 63999 (0xF9FF) ticks.
 	_tx->setDutyCycleB(_minEscCycle);
	_tx->setDutyCycleC(_minEscCycle);
	
	_ty->initialize(t_mode::PWM_F, t_channel::BC_TOP, t_inverted::NORMAL);	// TIMER3 same.
	_ty->setPrescaler(prescale);
	_ty->setCompareValueA(top);
	_ty->setDutyCycleB(_minEscCycle);
	_ty->setDutyCycleC(_minEscCycle);
	
	_tx->reset();								// Start PWM.
	_ty->reset();
}

/*******************************************************************************
 * Method for assigning a timer and PWM channel to an ESC. This should match
 * your hardware setup.
 * 
 * @param: alias The 16-bit timer alias.
 * @param: channel The 16-bit timer channel B or C. 
 ******************************************************************************/
int8_t ESCS::assign1(t_alias alias, t_channel channel)
{
	int8_t ret = 0;
	
	if(alias==_tx.getAlias())
	{
		if(channel==t_channel::B)setDutyCycle1 = &_tx->setDutyCycleB;
		else if(channel==t_channel::C)setDutyCycle1 = &_tx->setDutyCycleC;
		else{ret=-1;return ret;}
	}
	else if(alias==_ty.getAlias())
	{
		if(channel==t_channel::B)setDutyCycle1 = &_ty->setDutyCycleB;
		else if(channel==t_channel::C)setDutyCycle1 = &_ty->setDutyCycleC;
		else{ret=-1;return ret;}
	}
	else
	{
		ret = -1;
	}
	
	return ret;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t ESCS::assign2(t_alias alias, t_channel channel)
{
	int8_t ret = 0;
	
	if(alias==_tx.getAlias())
	{
		if(channel==t_channel::B)setDutyCycle2 = &_tx->setDutyCycleB;
		else if(channel==t_channel::C)setDutyCycle2 = &_tx->setDutyCycleC;
		else{ret=-1;return ret;}
	}
	else if(alias==_ty.getAlias())
	{
		if(channel==t_channel::B)setDutyCycle2 = &_ty->setDutyCycleB;
		else if(channel==t_channel::C)setDutyCycle2 = &_ty->setDutyCycleC;
		else{ret=-1;return ret;}
	}
	else
	{
		ret = -1;
	}
	
	return ret;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t ESCS::assign3(t_alias alias, t_channel channel)
{
	int8_t ret = 0;
	
	if(alias==_tx.getAlias())
	{
		if(channel==t_channel::B)setDutyCycle3 = &_tx->setDutyCycleB;
		else if(channel==t_channel::C)setDutyCycle3 = &_tx->setDutyCycleC;
		else{ret=-1;return ret;}
	}
	else if(alias==_ty.getAlias())
	{
		if(channel==t_channel::B)setDutyCycle3 = &_ty->setDutyCycleB;
		else if(channel==t_channel::C)setDutyCycle3 = &_ty->setDutyCycleC;
		else{ret=-1;return ret;}
	}
	else
	{
		ret = -1;
	}
	
	return ret;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t ESCS::assign4(t_alias alias, t_channel channel)
{
	int8_t ret = 0;
	
	if(alias==_tx.getAlias())
	{
		if(channel==t_channel::B)setDutyCycle4 = &_tx->setDutyCycleB;
		else if(channel==t_channel::C)setDutyCycle4 = &_tx->setDutyCycleC;
		else{ret=-1;return ret;}
	}
	else if(alias==_ty.getAlias())
	{
		if(channel==t_channel::B)setDutyCycle4 = &_ty->setDutyCycleB;
		else if(channel==t_channel::C)setDutyCycle4 = &_ty->setDutyCycleC;
		else{ret=-1;return ret;}
	}
	else
	{
		ret = -1;
	}
	
	return ret;
}

/*******************************************************************************
 * Method for writing variable PWM-pulses in length to the ESC's.
 *	
 * @param: dc1 The duty cycle for ESC1 in microseconds.
 ******************************************************************************/
void ESCS::writeSpeed1(float dc1)
{
	*setDutyCycle1(dc2Escc(dc1));						// Clockwise.
}

/*******************************************************************************
 * Method for writing variable PWM-pulses in length to the ESC's.
 * 
 * @param: dc2 The duty cycle for ESC2 in microseconds.
 ******************************************************************************/
void ESCS::writeSpeed2(float dc2)
{
	*setDutyCycle2(dc2Escc(dc2));						// Clockwise.
}

/*******************************************************************************
 * Method for writing variable PWM-pulses in length to the ESC's.
 * 
 * @param: dc2 The duty cycle for ESC2 in microseconds.
 ******************************************************************************/
void ESCS::writeSpeed3(float dc3)
{
	*setDutyCycle3(dc2Escc(dc3));						// Counter Clockwise.
}

/*******************************************************************************
 * Method for writing variable PWM-pulses in length to the ESC's.
 * 
 * @param: dc4 The duty cycle for ESC4 in microseconds.
 ******************************************************************************/
void ESCS::writeSpeed4(float dc4)
{
	*setDutyCycle4(dc2Escc(dc4));						// Counter Clockwise.
}

/*******************************************************************************
 * Method for writing variable PWM-pulses in length to the ESC's.
 *	
 * @param: dc1 The duty cycle for ESC1 in microseconds.
 * @param: dc2 The duty cycle for ESC2 in microseconds.
 * @param: dc3 The duty cycle for ESC3 in microseconds.
 * @param: dc4 The duty cycle for ESC4 in microseconds.
 ******************************************************************************/
void ESCS::writeSpeed(float dc1, float dc2, float dc3, float dc4)
{
	writeSpeed1(dc2Escc(dc1));
	writeSpeed2(dc2Escc(dc2));
	writeSpeed3(dc2Escc(dc3));
	writeSpeed4(dc2Escc(dc4));
}

/*******************************************************************************
 * Method for writing HIGH PWM-pulses to the ESC's.
 ******************************************************************************/
void ESCS::writeMaxSpeed()
{
	writeSpeed1(dc2Escc(_maxEscCycle));
	writeSpeed2(dc2Escc(_maxEscCycle));
	writeSpeed3(dc2Escc(_maxEscCycle));
	writeSpeed4(dc2Escc(_maxEscCycle));
}

/*******************************************************************************
 * Method for writing LOW PWM-pulses to the ESC's.
 ******************************************************************************/
void ESCS::writeMinSpeed()
{
	writeSpeed1(dc2Escc(_minEscCycle));
	writeSpeed2(dc2Escc(_minEscCycle));
	writeSpeed3(dc2Escc(_minEscCycle));
	writeSpeed4(dc2Escc(_minEscCycle));
}

/*******************************************************************************
 * Method for writing LOW PWM-pulses to the ESC's.
 ******************************************************************************/
float ESCS::dc2Escc(float dutyCycle)
{
	float esc;
	
	if(dutyCycle >= 1.0f)esc=_maxEscCycle;
	else if(dutyCycle <= 0.0f)esc=_minEscCycle;
	else{esc = (_maxEscCycle - _minEscCycle)*dutyCycle + _minEscCycle;}
	
	return esc;
}
