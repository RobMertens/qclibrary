/******************************************************************************
 * Quadcopter-Library-v1
 * ESC.hpp
 *
 * This file contains predefined functions for the ESC-class. A motor is
 * controlled by "asynchronous" Pulse Width Modulation (PWM). Asynchronous
 * points to the fact that the dutycycle can be set anywhere in the main
 * program. The motors are primarily controlled w/ hardware functions.
 *
 * Next sketch describes the quadcopter in top and bottom view. This sketch
 * describes the correct numbering and direction of rotation of each motor.
 *
 *	       TOP-VIEW			     		BOTTOM-VIEW
 *
 * 	/ 2 \   front   / 1 \		/ 2 \   front   / 1 \
 * 	\cw /           \ccw/		\ccw/           \cw /
 * 	   |::.........::|   		   |::.........::|
 * 	 l   |:       :|   r		 l   |:       :|   r
 * 	 e    |:     :|    i		 e    |:     :|    i
 * 	 f     |:::::|     g		 f     |:::::|     g
 * 	 t    |:     :|    h		 t    |:     :|    h
 * 	     |:       :|   t		     |:       :|   t
 * 	   |::.........::|		   |::.........::|
 * 	/ 3 \           / 4 \		/ 3 \           / 4 \
 * 	\ccw/   rear    \cw /		\cw /   rear    \ccw/

 * TODO::variable frequency and clock speed.
 *
 * @author: 	Rob Mertens
 * @date:	07/05/2017
 * @version: 	1.1.1
 ******************************************************************************/

#ifndef qc_ESC_HPP
#define qc_ESC_HPP

//Include standard headers.
#include <stdint.h>

//Include timer headers.
#include "timer.hpp"
#include "timer16.hpp"
#include "factory.hpp"

namespace qc
{

namespace component
{

/**
 * @brief
 */
class ESC
{
	public:
		/** Typedefs **************************************************************/
		/**
		 * @brief
		 */
		typedef ESC * Ptr; //typedef std::shared_ptr<ESC> ptr;

		/**
		 * @brief
		 */
		typedef ESC * const CPtr; //std::shared_ptr<ESC const> cptr;

		/** Constructors/destructors/overloading **********************************/
		/**
		 * @brief Constructor for the ESC-class. By making an object with this
		 *				constructor all local variables are set together with the
		 *				avr-timer.
		 *
		 * This constructor is compatible for most ESC's which have a DEFAULT
		 * microsecond PMW-range of minimum 1000us (motors off)	and maximum 2000us
		 * (motors at maximum throttle) or user specified.
		 *
		 * @param A Timer16 pointer. We explicitly want 16-bit timers since we need
		 *				two PWM-channels for two CW anc CCW motors.
		 * @param channel
		 * @param periodMicrosecond The period length [µs] (DEFAULT=4000).
		 * @param maxMicrosecond The full throttle pulse length [µs] (DEFAULT=2000).
		 * @param minMicrosecond The zero throttle pulse length [µs] (DEFAULT=1000).
		 */
		ESC(const avr::Timer16::Ptr&, const avr::t_channel&, const uint16_t=4000,
			const uint16_t=2000, const uint16_t=1000);

		/**
		 * @brief
		 * @param
		 */
		ESC(const ESC&);

		/**
		 * @brief
		 * @param
		 */
		virtual ~ESC(void);

		/**
		 * @brief
		 * @param
		 */
		ESC& operator=(const ESC&);

		/** ESC setting functions *************************************************/
		/**
		 * @brief Method for initializing the timer-settings.
		 * 				The default is TIMER1 with no prescaler and timer overflow interrupt.
		 * @param prescale The prescale mask value (DEFAULT=0x01).
		 */
		void arm(const uint16_t=0x0001, const uint16_t=0xF9FF);

		/**
		 * @brief TODO::safely shut down the motors.
		 */
		void unarm(void);

		/** ESC runtime functions *************************************************/
		/**
		 * @brief Method for writing variable PWM-pulses in length to the ESC's.
		 * @param dc The duty cycle for the ESC in microseconds.
		 * @return ret A return value for debugging ? 0 successful : -1 unsuccessful.
		 */
		int8_t writeSpeed(const double);

		/**
		 * @brief Method for writing HIGH PWM-pulses to the ESC's.
		 * @return ret A return value for debugging ? 0 successful : -1 unsuccessful.
		 */
		int8_t writeMaxSpeed(void);

		/**
		 * @brief Method for writing LOW PWM-pulses to the ESC's.
		 * @return ret A return value for debugging ? 0 successful : -1 unsuccessful.
		 */
		int8_t writeMinSpeed(void);

  private:

		/** Helper functions ******************************************************/
		/**
		 * @brief Method for assigning a timer and PWM channel to an ESC. This
		 *				should match your hardware setup.
		 *
		 * @param alias The 16-bit timer alias.
		 * @param channel The 16-bit timer channel B or C.
		 * @return ret A return value ? 0 for successful : -1 for unsuccessful.
		 */
    int8_t assign(const avr::t_channel&);

		/**
		 * @brief This function pointer points to the actual channel to which This
		 *				ESC is linked. The possibility is either channel B or C.
		 */
		int8_t (avr::Timer16::*setDutyCycle)(double);

		/**
		 * @brief Method for writing LOW PWM-pulses to the ESC's.
		 * @param
		 * @return
		 */
		double dc2Escc(const double);

		/** Variables *************************************************************/
		/**
		 * @brief A pointer to an Timer16 instance.
		 *
		 * We explicitly want a 16-bit timer because this has a PWM-channel
		 * possibility with three channels A, B and C. We need channel A to set the
		 * timer TOP count value. Hence the channels B and C are the base signals
		 * for two motors with the same rotational direction (CW) and (CCW).
		 *
		 * By doing so, we control two motors with one timer asynchronous from the
		 * main computation loop.
		 */
		avr::Timer16::Ptr tptr_;

		/**
		 * @brief
		 */
		static double maxEscCycle_;																													// Maximum timerticks range.

		/**
		 * @brief
		 */
		static double minEscCycle_;																													// Minimum timerticks range.

}; //End ESC class.

}; //End namespace component.

}; //End namespace qc.

#endif //End qc_ESC_HPP file.
