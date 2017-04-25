/******************************************************************************
 * Quadcopter-Library-v1
 * LED.cpp
 *  
 * This file contains predefined functions for the LED-class.
 * 
 * TODO::support multiple LED's.
 * 
 * @author Rob Mertens
 * @version 1.0.1 14/08/2016
 ******************************************************************************/

#include <LED.h>

/*******************************************************************************
 * Constructor for the LED-class.
 ******************************************************************************/
LED::LED(volatile uint8_t * ddr, uint8_t ddrmsk, volatile uint8_t * pin, volatile uint8_t * port)
{	
	_ddr	= ddr;
	*_ddr	|= ddrmsk;							// Put definitions in local variables.
	_pin	= pin;
	_port	= port;
	
	_HIGH	= *_ddr;
	_LOW	= *_ddr ^ 0xFF;
}

/*******************************************************************************
 * Method for turning the LED-state on.
 ******************************************************************************/
void LED::setLed()
{
	*_port |= _HIGH;
}

/*******************************************************************************
 * Method for turning the LED-state off.
 ******************************************************************************/
void LED::resetLed()
{
	*_port &= _LOW;
}

/*******************************************************************************
 * Method for turning the LED-state off.
 ******************************************************************************/
void LED::toggleLed()
{
	*_port ^= 0xFF;
}

/*******************************************************************************
 * Method for turning the LED-state off. Used for debugging.
 *
 * @return state The current LED-state (TRUE/FALSE).
 ******************************************************************************/
bool LED::getLedState()
{
	bool state = false;
	
	if(*_pin & 0x80)state = true;
	
	return state;
}
