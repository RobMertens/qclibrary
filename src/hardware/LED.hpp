/******************************************************************************
 * Quadcopter-Library-v1
 * LED.hpp
 *
 * This file contains predefined functions for the LED-class.
 *
 * @author 	Rob Mertens
 * @date 		14/08/2016
 * @version 1.1.1
 ******************************************************************************/

#ifndef qc_LED_HPP
#define qc_LED_HPP

//Include standard headers.
#include <stdint.h>

namespace qc
{

namespace component
{

/**
 * @brief
 */
class LED
{
	public:
		/** Typedefs **************************************************************/
		/**
		 * @brief Pointer typedef to LED instance.
		 */
		typedef LED * Ptr;

		/**
		 * @brief Constant pointer typedef to LED instance.
		 */
		typedef LED * const Cptr;

		/** Constructors/destructors/overloading **********************************/
		/**
		 * @brief Main constructor for a LED instance.
		 * @param Data Direction Register (DDR) port byte pointer.
		 * @param Data Direction Register Mask (DDRMSK) byte.
		 * @param Pin Input (PIN) register byte pointer.
		 * @param Pin Output (PORT) register byte pointer.
		 */
		LED(const volatile uint8_t * const&, const uint8_t,
			const volatile uint8_t * const&, const volatile uint8_t * const&);

		/**
		 * @brief Copy-constructor.
		 */
		LED(const LED&);

		/**
		 * @brief Destructor.
		 */
		virtual ~LED(void);

		/**
		 * @brief Object equals operator overloading.
		 */
		LED& operator=(const LED&);

		/** Basic functions *******************************************************/
		/**
		 * @brief Method for turning the LED-state on.
		 */
		void set(void);

		/**
		 * @brief Method for turning the LED-state off.
		 */
		void reset(void);

		/**
		 * @brief Method for switching the LED-state.
		 */
		void toggle(void);

		/**
		 * @brief Method for getting the actual LED-state (debugging).
		 * @return The actual LED state ? true [ON] : false [OFF].
		 */
		bool getState(void);

	private:

		//Registers ****************************************************************
		/**
		 * @brief The local data direction register pointer.
		 * 				Data Direction Register (ex.: DDRC 0b00001111).
		 */
		volatile uint8_t * ddr_;

		/**
		 * @brief The local input pin register pointer for reading the LED-state.
		 */
		volatile uint8_t * pin_;

		/**
		 * @brief The local output pin register pointer for writing the LED-state.
		 */
		volatile uint8_t * port_;

		//Variables ****************************************************************
		/**
		 * @brief Variable with HIGH LED-state.
		 */
		static uint8_t HIGH_;

		/**
		 * @brief Variable with LOW LED-state.
		 */
		static uint8_t LOW_;

}; //End LED class.

}; //End component namespace.

}; //End qc namespace.

#endif //End qc_LED_HPP file.
