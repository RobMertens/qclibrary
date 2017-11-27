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

#include "hardware/BAT.hpp"

namespace qc
{

namespace component
{

BAT::BAT(void)
{
	level_ = 0x00;
}

uint8_t BAT::getLevel(void)
{
	//TODO::measurement.

	return level_;
}

}; //End namespace component.

}; //End namespace qc.
