/******************************************************************************
 * Quadcopter-Library-v1
 * BAT.hpp
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

#ifndef qc_BAT_HPP
#define qc_BAT_HPP

//Include header.
#include <stdint.h>

namespace qc
{

namespace component
{

/**
 * @brief
 */
class BAT
{
	public:
		/** Typedefs **************************************************************/
		/**
		 * @brief
		 */
		typedef BAT * Ptr; //std::shared_ptr<BAT> ptr;

		/**
		 * @brief
		 */
		typedef BAT * const CPtr; //std::shared_ptr<BAT const> cptr;

		/** Constructors **********************************************************/
		/**
		 * @brief
		 */
		BAT(void);

		//Setters ********************************************************************

		//Getters *****************************************************************/
		/**
		 * @brief
		 * @return
		 */
		uint8_t getLevel(void);

	private:

		/**
		 * @brief Variable with LOW LED-state.
		 */
		uint8_t level_;

}; //End class BAT.

}; //

}; //

#endif //End wrapper qc_BAT_HPP.
