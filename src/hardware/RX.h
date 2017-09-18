#ifndef _RX_H_
#define _RX_H_

//#include <memory>
#include <stdint.h>
#include <avr/interrupt.h>
#include "interrupt.h"
#include "timer8.h"

extern "C" void PCINT0_vect(void) __attribute__ ((signal));
extern "C" void PCINT1_vect(void) __attribute__ ((signal));
extern "C" void PCINT2_vect(void) __attribute__ ((signal));
extern "C" void PCINT3_vect(void) __attribute__ ((signal));

namespace rx_settings
{

	enum class mode : uint8_t
	{
		NONE = 0,
		M1 = 1,
		M2 = 2
	};

}; //End namespace rx_settings.

class RX : public interrupt::handler
{
  public:
		//Typedefs *****************************************************************
		typedef RX * ptr;//std::shared_ptr<RX> ptr;
		typedef RX * const cptr;//std::shared_ptr<RX const> cptr;

	  //Constructors *************************************************************
	  RX(const volatile uint8_t * const&, const uint8_t, const t_settings::alias, const uint16_t=4000, const uint16_t =2000, const uint16_t=1000);

	  //Setters ******************************************************************
	  void initialize(const rx_settings::mode);
	  virtual void interruptServiceRoutine(void);
		virtual void enable(void);
		virtual void disable(void);
		virtual void clear(void);

		//Getters ******************************************************************
		int8_t setMode(rx_settings::mode);
		int8_t assignThrottleChannel(uint8_t);
		int8_t assignPitchChannel(uint8_t);
		int8_t assignRollChannel(uint8_t);
		int8_t assignYawChannel(uint8_t);
		int8_t assignExtraChannel(void);
		uint8_t getThrottleChannel(void);
		uint8_t getPitchChannel(void);
		uint8_t getRollChannel(void);
		uint8_t getYawChannel(void);
		uint8_t getExtraChannel(void);
		float getThrottleInput(void);
		float getPitchInput(void);
		float getRollInput(void);
		float getYawInput(void);
		float getExtraInput(void);

	private:
		//Register *****************************************************************
		volatile uint8_t * _pcmskx;
		volatile uint8_t * _pin;
		uint8_t _pcie;
		uint8_t _ch1;
		uint8_t _ch2;
		uint8_t _ch3;
		uint8_t _ch4;
		uint8_t _ch5;
		uint8_t _lastChannel;

		timer8 _t;
		rx_settings::mode _mode;

		//Channel mapping **********************************************************
		uint32_t * _channelT;
		uint32_t * _channelR;
		uint32_t * _channelP;
		uint32_t * _channelY;
		uint32_t * _channelE;
		uint8_t _chT;											// Link to number.
		uint8_t _chR;
		uint8_t _chP;
		uint8_t _chY;
		uint8_t _chE;

		//Channel variables ********************************************************
		uint32_t _channel1;								// Double word for storing the actual timer value.
		uint32_t _channel2;
		uint32_t _channel3;
		uint32_t _channel4;
		uint32_t _channel5;
		int8_t _offset1;
		int8_t _offset2;
		int8_t _offset3;
		int8_t _offset4;
		int8_t _offset5;

		//Static values ************************************************************
		uint8_t _periodMicroseconds;
		float _maxRxCycle;								// Maximum timerticks range.
		float _minRxCycle;								// Minimum timerticks range.
		//static const float _DEADBAND = 2.0f;
		//static const float _MAXROLL = 100.0f;

		//Setters ******************************************************************

		//Getters ******************************************************************
		float rxc2dc(const float, const float);

		//Interrupt functions ******************************************************
		static RX::ptr _RX[4];
		friend void PCINT0_vect(void);
		#if defined(PCINT1_vect)
		friend void PCINT1_vect(void);
		#endif
		#if defined(PCINT2_vect)
		friend void PCINT2_vect(void);
		#endif
		#if defined(PCINT3_vect)
		friend void PCINT3_vect(void);
		#endif

};
#endif
