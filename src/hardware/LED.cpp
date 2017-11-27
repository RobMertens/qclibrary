/******************************************************************************
 * Quadcopter-Library-v1
 * LED.cpp
 *
 * This file contains predefined functions for the LED-class.
 *
 * @author Rob Mertens
 * @version 1.0.1 14/08/2016
 ******************************************************************************/

#include "hardware/LED.hpp"

namespace qc
{

namespace component
{

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
	ddr_	= ddr;
	*ddr_	|= ddrmsk;							// Put definitions in local variables.
	pin_	= pin;
	port_	= port;

	HIGH_	= *ddr_;
	LOW_	= *ddr_ ^ 0xFF;
}

/*******************************************************************************
 *
 ******************************************************************************/
LED::LED(const LED& other)
{
	ddr_ = new volatile uint8_t(*other.ddr_);
	pin_ = new volatile uint8_t(*other.pin_);
	port_ = new volatile uint8_t(*other.port_);
}

/*******************************************************************************
 *
 ******************************************************************************/
LED::~LED(void)
{
	delete ddr_;
	delete pin_;
	delete port_;
}

/*******************************************************************************
 *
 ******************************************************************************/
LED& LED::operator=(const LED& other)
{
	if(this != &other)
	{
		//Volatiles.
		uint8_t *ddr(new volatile uint8_t(*other.ddr_));
		delete ddr_;
		ddr_ = ddr;
		uint8_t *pin(new volatile uint8_t(*other.pin_));
		delete pin_;
		pin_ = pin;
		uint8_t *port(new volatile uint8_t(*other.port_));
		delete port_;
		port_ = port;
	}
	return (*this);
}

/*******************************************************************************
 * @brief Method for turning the LED-state on.
 ******************************************************************************/
void LED::set(void)
{
	*port_ |= HIGH_;
}

/*******************************************************************************
 * @brief Method for turning the LED-state off.
 ******************************************************************************/
void LED::reset(void)
{
	*port_ &= LOW_;
}

/*******************************************************************************
 * @brief Method for turning the LED-state off.
 ******************************************************************************/
void LED::toggle(void)
{
	*port_ ^= 0xFF;
}

/*******************************************************************************
 * @brief Method for turning the LED-state off. Used for debugging.
 * @return state The current LED-state (TRUE/FALSE).
 ******************************************************************************/
bool LED::getState(void)
{
	bool state = false;

	if(*pin_ & 0x80)state = true;

	return state;
}

}; //End component namespace.

}; //End qc namespace.
