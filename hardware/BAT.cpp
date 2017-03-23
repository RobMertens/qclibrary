/******************************************************************************
 *	Quadcopter-Library-v1
 *  BAT.cpp
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

#include <BAT.h>

/*******************************************************************************
 * 	Constructor for the LED-class.
 ******************************************************************************/
BAT::BAT(void)
{	
	_level = 0x00;
}

/*******************************************************************************
 * 	Constructor for the LED-class.
 ******************************************************************************/
void BAT::measureBatteryLevel(void)
{
	
}

/*******************************************************************************
 * 	Constructor for the LED-class.
 ******************************************************************************/
uint8_t BAT::getBatteryLevel(void)
{
	return _level;
}

