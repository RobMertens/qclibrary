#ifndef _RX_H_
#define _RX_H_

#include <stdint.h>
#include <avr/interrupt.h>
#include "interrupt.h"
#include "timer8.h"

extern "C" void PCINT0_vect(void) __attribute__ ((signal));
extern "C" void PCINT1_vect(void) __attribute__ ((signal));
extern "C" void PCINT2_vect(void) __attribute__ ((signal));
extern "C" void PCINT3_vect(void) __attribute__ ((signal));

enum class rx_mode : uint8_t
{
	NONE = 0,
	M1 = 1,
	M2 = 2
};

class RX : public interrupt::handler
{

  	public:
	  	//Constructors ***************************************************************
	    	RX(volatile uint8_t *, uint8_t, t_alias, uint16_t=4000, uint16_t =2000, uint16_t=1000);
	    
	    	//Setters ********************************************************************
	   	void initialize(rx_mode);
	    
	    	virtual void interruptServiceRoutine(void);
		virtual void enable(void);
		virtual void disable(void);
		virtual void clear(void);
	    
	    	//Getters ********************************************************************
		int8_t setMode(rx_mode);
		int8_t assignThrottleChannel(uint8_t);
		int8_t assignPitchChannel(uint8_t);
		int8_t assignRollChannel(uint8_t);
		int8_t assignYawChannel(uint8_t);
		
		float getThrottleChannel();
		float getPitchChannel();
		float getRollChannel();
		float getYawChannel();
		uint8_t getExtraChannel();
	
	private:
		//Register *******************************************************************		
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
		
		rx_mode _mode;
		
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
		
		float rxc2dc(float);
		
		void setMode2M1(void);
		void setMode2M2(void);
		
		// Static self.
		static RX * _RX[4];
	
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
