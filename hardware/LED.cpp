/******************************************************************************
 *	Quadcopter-Library-v1
 *  LED.cpp
 *  
 *	This file contains predefined functions for the LED-class. These functions
 *	are very basic and just made for integraty.
 *
 *	In this class predefined registers are used. For each hardware-setup
 *	these registers can be different. It is recommended to change the registers
 *	in the DEFINE.h file.
 *
 *  @author Rob Mertens
 *  @version 1.0.1 14/08/2016
 ******************************************************************************/

#include <LED.h>

/*******************************************************************************
 * 	Constructor for the LED-class.
 ******************************************************************************/
LED::LED()
{	
	_ddr  = &LED_DDR;						// Put definitions in local variables.
	_pin  = &LED_PIN;
	_port = &LED_PORT;
	
    _HIGH = *_ddr;
    _LOW  = *_ddr ^ 0xFF;
    
}

/*******************************************************************************
 * 	Method for turning the LED-state on.
 ******************************************************************************/
void LED::setLedStateHigh()
{
	*_port |= _HIGH;
}

/*******************************************************************************
 * 	Method for turning the LED-state off.
 ******************************************************************************/
void LED::setLedStateLow()
{
	*_port &= _LOW;
}

/*******************************************************************************
 * 	Method for turning the LED-state off.
 ******************************************************************************/
void LED::invertLedState()
{
	*_port ^= 0xFF;
}

/*******************************************************************************
 * 	Method for turning the LED-state off. Used for debugging.
 *
 *	@return state The current LED-state (TRUE/FALSE).
 ******************************************************************************/
bool LED::getLedState()
{
	bool state = false;
	
	if(*_pin & 0x80)state = true;
	
	return state;
}