/******************************************************************************
 * Quadcopter-Library-v1
 * BAT.cpp
 *  
 * This file contains the functions for the battery (BAT). The battery-level
 * is measured by a simple analogue signal. This requires a simple voltage
 * divider in the hardware setup.
 *
 * TODO::support LiPo and Ni??
 * TODO::support different battery cell numbers.
 *
 * @author:	Rob Mertens
 * @date:	14/08/2016
 * @version:	1.0.1
 ******************************************************************************/

#include <BAT.h>

/*******************************************************************************
 * Constructor for the LED-class.
 ******************************************************************************/
BAT::BAT(void)
{	
	_level = 0x00;
}

/*******************************************************************************
 * Method for measuring the battery level.
 ******************************************************************************/
void BAT::measureBatteryLevel()
{
	//TODO::read out the analogue signal and map this to a percentage.
	//	This relation is non-linear (see discharge curve LiPo).
}

/*******************************************************************************
 * Constructor for the LED-class.
 ******************************************************************************/
uint8_t BAT::getBatteryLevel()
{
	return _level;
}

