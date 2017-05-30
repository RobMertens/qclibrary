/******************************************************************************
 * Quadcopter-Library-v1
 * PID.cpp
 *  
 * This file contains predefined functions for the PID-class. This PID-
 * controller is a parallel circuit of three controllers (P,I and D).
 * 
 * @author	Rob Mertens
 * @date	14/08/2016
 * @version	1.1.1
 ******************************************************************************/

#include "src/PID.h"

/*******************************************************************************
 * Constructor for the PID-controller.
 ******************************************************************************/
PID::PID(void)
{	    
	_iterm = 0;
	_lastError = 0;
}

/*******************************************************************************
 * Constructor for the PID-controller.
 * 	
 * @param kp The proportional gain.
 * @param ki The integral gain.
 * @param kd The derivative gain.
 * @param direction The direction of the controller (FORWARD/REVERSE).
 * @param minLimit The minimum output limit.
 * @param maxLimit The maximum output limit.
 ******************************************************************************/
PID::PID(float kp, float ki, float kd, float maxLimit, float minLimit, int8_t direction)
{	
	_kp = kp;
	_ki = ki;
	_kd = kd;

	_direction = direction;

	_maxLimit = maxLimit;
	_minLimit = minLimit;

	_iterm = 0;
	_lastError = 0;
}

/*******************************************************************************
 * Method for setting the minimum and maximum output limit.
 * 
 * @param: minLimit The minimum output limit.
 * @param: maxLimit The maximum output limit.
 ******************************************************************************/
void PID::setOutputLimits(float maxLimit, float minLimit)
{
	_maxLimit = maxLimit;
	_minLimit = minLimit;
}

/*******************************************************************************
 * Method for setting the proportional, integral and derivative gain values.
 * 
 * @param kp The proportional gain.
 * @param ki The integral gain.
 * @param kd The derivative gain.
 ******************************************************************************/
void PID::setGainValues(double kp, double ki, double kd)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
}

/*******************************************************************************
 * Method for changing the direction of the controller (FORWARD/REVERSE).
 * 
 * @param direction The direction of the controller (FORWARD/REVERSE).
 ******************************************************************************/
void PID::setDirection(int direction)
{
	_direction = direction;
}

/*******************************************************************************
 * Method for resetting the PID-controller.
 ******************************************************************************/
void PID::reset()
{
	_iterm = 0.0;
	_lastError = 0.0;
}

/*******************************************************************************
 * Method for calculating the output of the controller.
 * 
 * @param input The current input value.
 * @param desired The desired input value.
 ******************************************************************************/
float PID::calculate(float input, float desired)
{
	_input = input;
	_desired = desired;

	float error = _input - _desired;
	_iterm += _ki*error;

	_output = _kp*error + _iterm*error + _kd*(error - _lastError);
	if(_output > _maxLimit)_output = _maxLimit;
	else if(_output < _minLimit)_output = _minLimit;
	_output *= _direction;
	_lastError = error;

	return _output;
}

/*******************************************************************************
 * Method for getting the minimum output limit.
 * 
 * @return _minLimit The minimum output limit.
 ******************************************************************************/
float PID::getMinOutputLimit()
{
	return _minLimit;
}

/*******************************************************************************
 * Method for getting the maximum output limit.
 * 
 * @return _maxLimit The maimum output limit.
 ******************************************************************************/
float PID::getMaxOutputLimit()
{
	return _maxLimit;
}

/*******************************************************************************
 * Method for getting the proportional gain value.
 * 
 * @return _kp The proportional gain value.
 ******************************************************************************/
float PID::getProportionalGain()
{
	return _kp;
}

/*******************************************************************************
 * Method for getting the integral gain value.
 * 
 * @return _ki The integral gain value.
 ******************************************************************************/
float PID::getIntegralGain()
{
	return _ki;
}

/*******************************************************************************
 * Method for getting the derivative gain value.
 * 
 * @return _kd The derivative gain value.
 ******************************************************************************/
float PID::getDifferentialGain()
{
	return _kd;
}

/*******************************************************************************
 * Method for getting the direction of the controller.
 * 
 * @return _direction The direction of the controller.
 ******************************************************************************/
uint8_t PID::getDirection()
{
	return _direction ? FORWARD : REVERSE;
}
