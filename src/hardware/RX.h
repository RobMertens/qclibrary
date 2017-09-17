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
		float getThrottleChannel(void);
		float getPitchChannel(void);
		float getRollChannel(void);
		float getYawChannel(void);
		uint8_t getExtraChannel(void);

	private:
		//Register *****************************************************************
		volatile uint8_t * _pcmskx;
		volatile uint8_t * _pin;
		uint8_t _pcie;
		uint8_t _ch1;
		uint8_t _ch2;
		uint8_t _ch3;
		uint8_t _ch4;
		timer8 _t;
		uint8_t _periodMicroseconds;
		float _maxRxCycle;								// Maximum timerticks range.
	  float _minRxCycle;								// Minimum timerticks range.
		rx_settings::mode _mode;
		int8_t _offsetCh1;
		int8_t _offsetCh2;
		int8_t _offsetCh3;
		int8_t _offsetCh4;
		uint32_t _channel1;								// Double word for storing the actual timer value.
		uint32_t _channel2;
		uint32_t _channel3;
		uint32_t _channel4;
		uint32_t * _throttleChannel;
		uint32_t * _rollChannel;
		uint32_t * _pitchChannel;
		uint32_t * _yawChannel;
		uint8_t _lastChannel;
		//static const float _DEADBAND = 2.0f;
		//static const float _MAXROLL = 100.0f;

		//Setters ******************************************************************
		void setMode2M1(void);
		void setMode2M2(void);

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
