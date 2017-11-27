/******************************************************************************
 * Quadcopter-Library-v1
 * RX.hpp
 *
 * @author: 	Rob Mertens
 * @date:			14/08/2016
 * @version: 	1.1.1
 ******************************************************************************/

#ifndef qc_RX_HPP
#define qc_RX_HPP

//Include standard headers.
#include <stdint.h>
#include <avr/interrupt.h>

//Include additional headers.
#include "interrupt.hpp"
#include "timer.hpp"
#include "timer8.hpp"
#include "timer16.hpp"
#include "factory.hpp"

//Include qc headers.
//None.

namespace qc
{

namespace component
{

/**
 * @brief Extern C pin change interrupt vector 0.
 */
extern "C" void PCINT0_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C pin change interrupt vector 1.
 */
extern "C" void PCINT1_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C pin change interrupt vector 2.
 */
extern "C" void PCINT2_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C pin change interrupt vector 3.
 */
extern "C" void PCINT3_vect(void) __attribute__ ((signal));

/**
 * @brief RX class for instances of a 5 channel receiver.
 */
class RX : public avr::Interrupt::Handler
{
  public:
		/** Typedefs **************************************************************/
		/**
		 * @brief Pointer to receiver instance.
		 *
		 * TODO::work with smart pointers, e.g. std/boost::shared_ptr.
		 */
		typedef RX * Ptr;

		/**
		 * @brief Const pointer to receiver instance.
		 *
		 * TODO::work with smart pointers, e.g. std/boost::shared_ptr.
		 */
		typedef RX * const Cptr;

		/** Enum ******************************************************************/
		/**
		 * @brief Enum describing the controller mode. The mode describes joystick
		 * 				to channel mapping.
		 *
		 * [None] No mapping.
		 * [M1]   Mode 1, e.g. standardized joystick mapping.
		 * [M2]   Mode 2,
		 * [MX] 	IDEA::user defined mode??
		 */
		enum Mode { NONE=0, M1=1, M2=2 };

		/** Constructors/destructors/overloads ************************************/
		/**
		 * @brief Constructor for a receiver instance.
		 * @param
		 * @param
		 * @param
		 * @param
		 * @param
		 * @param
		 */
	  RX(const volatile uint8_t * const&, const uint8_t,
			const avr::Timer::Ptr&, const uint16_t=4000, const uint16_t =2000,
			const uint16_t=1000);

		/**
		 * @brief Copy-constructor for a receiver instance.
		 * @param
		 */
		RX(const RX&) = delete;

		/**
		 * @brief Destructor for a receiver instance.
		 */
		virtual ~RX(void);

		/**
		 * @brief Operator overloading.
		 *
		 * NOTE::moved private.
		 */
		//RX& operator=(const RX&) = delete;

		/** RX settings functions *******************************************/
		/**
		 * @brief
		 */
	  void initialize(const Mode&);

		/**
		 * @brief Method for setting the receiver mode.
		 */
		int8_t setMode(const Mode&);

		/**
		 * @brief Method for assigning the throttle channel to a channel number.
		 * @param The channel number byte.
		 * @return The signed return byte ? succes 0 : failure -1.
		 */
		int8_t assignThrottleChannel(const uint8_t);

		/**
		 * @brief Method for assigning the pitch channel to a channel number.
		 * @param The channel number byte.
		 * @return The signed return byte ? succes 0 : failure -1.
		 */
		int8_t assignPitchChannel(const uint8_t);

		/**
		 * @brief Method for assigning the roll channel to a channel number.
		 * @param The channel number byte.
		 * @return The signed return byte ? succes 0 : failure -1.
		 */
		int8_t assignRollChannel(const uint8_t);

		/**
		 * @brief Method for assigning the yaw channel to a channel number.
		 * @param The channel number byte.
		 * @return The signed return byte ? succes 0 : failure -1.
		 */
		int8_t assignYawChannel(const uint8_t);

		/**
		 * @brief Method for assigning the extra channel to a channel number.
		 * @param The channel number byte.
		 * @return The signed return byte ? succes 0 : failure -1.
		 */
		int8_t assignExtraChannel(void);

		/** RX runtime functions ********************************************/
		/**
		 * @brief Method for assigning the extra channel to a channel number.
		 *				TODO::add functions for this channel.
		 * @return The signed return byte ? succes 0 : failure -1.
		 */
		uint8_t getThrottleChannel(void);

		/**
		 * @brief Method for assigning the extra channel to a channel number.
		 * @return The channel number byte.
		 */
		uint8_t getPitchChannel(void);

		/**
		 * @brief Method for assigning the extra channel to a channel number.
		 * @return The channel number byte.
		 */
		uint8_t getRollChannel(void);

		/**
		 * @brief Method for assigning the extra channel to a channel number.
		 * @return The channel number byte.
		 */
		uint8_t getYawChannel(void);

		/**
		 * @brief Method for assigning the extra channel to a channel number.
		 * @return The channel number byte.
		 */
		uint8_t getExtraChannel(void);

		/**
		 * @brief Method for getting the throttle value in duty cycle.
		 * @return The throttle input duty cycle.
		 */
		double getThrottleInput(void);

		/**
		 * @brief Method for getting the pitch value in duty cycle.
		 * @return The pitch input duty cycle.
		 */
		double getPitchInput(void);

		/**
		 * @brief Method for getting the roll value in duty cycle.
		 * @return The roll input duty cycle.
		 */
		double getRollInput(void);

		/**
		 * @brief Method for getting the yaw value in duty cycle.
		 * @return The yaw input duty cycle.
		 */
		double getYawInput(void);

		/**
		 * @brief Method for getting the extra value in duty cycle.
		 *				IDEA::map to discrete/continous value.
		 * @return The extra input duty cycle.
		 */
		double getExtraInput(void);

		/** Overloading interrupt functions ***************************************/
 		/**
 		 * @brief Method overwriting the interrupt enable function in the interrupt
 		 *				class.
 		 */
 		void enable(void) override;

 		/**
 		 * @brief Method overwriting the interrupt disable function in the interrupt
 		 *				class.
 		 */
 		void disable(void) override;

 		/**
 		 * @brief Method overwriting the interrupt clear function in the interrupt
 		 *				class.
 		 */
 		void clear(void) override;


	private:

		/** Constructors/destructors/overloads ************************************/
		/**
		 * @brief Operator overloading.
		 *
		 * NOTE::moved private.
		 */
		RX& operator=(const RX&);

		/** Overloading interrupt functions ***************************************/
		/**
 		 * @brief Method overwriting the interrupt service routine (ISR) in the
 		 * 				interrupt class.
 		 */
 		void interruptServiceRoutine(void) override;

		/** Helper functions ******************************************************/
		/**
		 * @brief Helper function for the conversion betweer receiver cycles and
		 * 				duty cycles.
		 *
		 * The receiver cycle is a signal expressed as PWM pulses to fire the ESCs.
		 * This is standard output of most receivers.
		 * The duty cycle is a generalized signal which varies between zero and one.
		 *
		 * @param The maximum PWM pulses cycle.
		 * @param The maximum duty cycle value, standard value 1.0.
		 * @return The actual duty cycle.
		 */
		double rxc2dc(const double, const double=1.0);

		/** Registers *************************************************************/
		/**
		 * @brief Pin Change Mask register (PCMSK) pointer.
		 */
		volatile uint8_t * pcmskx_;

		/**
		 * @brief Pin Change input register (PIN) pointer.
		 */
		volatile uint8_t * pin_;

		/**
		 * @brief Pin Change Interrupt ????.
		 */
		uint8_t pcie_;

		/**
		 * @brief Local instance timer to
		 */
		avr::Timer::Ptr tptr_;

		/**
		 * @brief Pointer to the throttle (T) channel value.
		 */
		Mode mode_;

		/** Channel mapping *******************************************************/
		/**
		 * @brief Pointer to the throttle (T) channel value.
		 */
		uint32_t * channelT_;

		/**
		 * @brief Pointer to the roll (R) channel value.
		 */
		uint32_t * channelR_;

		/**
		 * @brief Pointer to the pitch (P) channel value.
		 */
		uint32_t * channelP_;

		/**
		 * @brief Pointer to the yaw (Y) channel value.
		 */
		uint32_t * channelY_;

		/**
		 * @brief Pointer to the extra (E) channel value.
		 */
		uint32_t * channelE_;

		/** Channel pin change interrupt location *********************************/
		/**
		 * @brief Pin change location is channel one.
		 */
		uint8_t ch1_;

		/**
		 * @brief Pin change location is channel two.
		 */
		uint8_t ch2_;

		/**
		 * @brief Pin change location is channel three.
		 */
		uint8_t ch3_;

		/**
		 * @brief Pin change location is channel four.
		 */
		uint8_t ch4_;

		/**
		 * @brief Pin change location is channel five.
		 */
		uint8_t ch5_;

		/**
		 * @brief Remembers which channels are set high.
		 */
		uint8_t lastChannel_;

		/** Channel variables *****************************************************/
		/**
		 * @brief Channel one double word data container.
		 */
		uint32_t channel1_;

		/**
		 * @brief Channel two double word data container.
		 */
		uint32_t channel2_;

		/**
		 * @brief Channel three double word data container.
		 */
		uint32_t channel3_;

		/**
		 * @brief Channel four double word data container.
		 */
		uint32_t channel4_;

		/**
		 * @brief Channel five double word data container.
		 */
		uint32_t channel5_;

		/**
		 * @brief Channel one fixed offset.
		 */
		int16_t offset1_;

		/**
		 * @brief Channel two fixed offset.
		 */
		int8_t offset2_;

		/**
		 * @brief Channel three fixed offset.
		 */
		int8_t offset3_;

		/**
		 * @brief Channel four fixed offset.
		 */
		int8_t offset4_;

		/**
		 * @brief Channel five fixed offset.
		 */
		int8_t offset5_;

		/** Static values *********************************************************/
		/**
		 * @brief
		 */
		static uint8_t periodMicroseconds_;

		/**
		 * @brief Maximum timerticks range.
		 */
		static double maxRxCycle_;

		/**
		 * @brief Minimum timerticks range.
		 */
		static double minRxCycle_;
		//static const double _DEADBAND = 2.0f;
		//static const double _MAXROLL = 100.0f;

		/** Interrupt functions ***************************************************/
		/**
		 * @brief Static receiver instance for each interrupt vector.
		 * 				QUESTION::is this correct -> should be _globals_ (see .cpp)?
		 */
		static RX::Ptr __RX__[4];

		/**
		 * @brief Friend interrupt zero vector. This vector is standard for each
		 * 				arduino and thus standard friend.
		 */
		friend void PCINT0_vect(void);

		/**
		 * @brief Friend interrupt one vector.
		 */
		#if defined(PCINT1_vect)
		friend void PCINT1_vect(void);
		#endif

		/**
		 * @brief Friend interrupt two vector.
		 */
		#if defined(PCINT2_vect)
		friend void PCINT2_vect(void);
		#endif

		/**
		 * @brief Friend interrupt three vector.
		 */
		#if defined(PCINT3_vect)
		friend void PCINT3_vect(void);
		#endif
};

}; //End namespace component.

}; //End namespace QC.

#endif //End file qc_RX_HPP.
