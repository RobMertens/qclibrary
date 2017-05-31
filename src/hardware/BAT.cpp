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
 * @version:	1.1.1
 ******************************************************************************/

#include "hardware/BAT.h"

/*******************************************************************************
 * Constructor for the LED-class.
 ******************************************************************************/
BAT::BAT(void)
{	
	_level = 0x00;
}

/*******************************************************************************
 * Constructor for the LED-class.
 ******************************************************************************/
uint8_t BAT::getLevel()
{
	//TODO::measurement.
		
	return _level;
}

