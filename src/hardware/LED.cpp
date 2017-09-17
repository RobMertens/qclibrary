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

#include "hardware/LED.h"

/*******************************************************************************
 * @brief Constructor for the LED-class.
 * @param ddr
 * @param ddrmsk
 * @param pin
 * @param port
 ******************************************************************************/
LED::LED(const volatile uint8_t * const& ddr,
				 const uint8_t ddrmsk,
				 const volatile uint8_t * const& pin,
				 const volatile uint8_t * const& port)
{
	_ddr	= ddr;
	*_ddr	|= ddrmsk;							// Put definitions in local variables.
	_pin	= pin;
	_port	= port;

	_HIGH	= *_ddr;
	_LOW	= *_ddr ^ 0xFF;
}

/*******************************************************************************
 * @brief Method for turning the LED-state on.
 ******************************************************************************/
void LED::set(void)
{
	*_port |= _HIGH;
}

/*******************************************************************************
 * @brief Method for turning the LED-state off.
 ******************************************************************************/
void LED::reset(void)
{
	*_port &= _LOW;
}

/*******************************************************************************
 * @brief Method for turning the LED-state off.
 ******************************************************************************/
void LED::toggle(void)
{
	*_port ^= 0xFF;
}

/*******************************************************************************
 * @brief Method for turning the LED-state off. Used for debugging.
 * @return state The current LED-state (TRUE/FALSE).
 ******************************************************************************/
bool LED::getState(void)
{
	bool state = false;

	if(*_pin & 0x80)state = true;

	return state;
}
