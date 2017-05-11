#ifndef _RX_H_
#define _RX_H_

#include <avr/interrupt.h>
#include "interrupt.h"

class timer8;

extern "C" void PCINT0_vect(void) __attribute__ ((signal));
extern "C" void PCINT1_vect(void) __attribute__ ((signal));
extern "C" void PCINT2_vect(void) __attribute__ ((signal));
extern "C" void PCINT3_vect(void) __attribute__ ((signal));

class RX : public interrupt::handler
{

  	public:
	  	//Constructors ***************************************************************
	    	RX(volatile uint8_t *, volatile uint8_t *, uint8_t, uint8_t, uint16_t=4000, uint16_t =2000, uint16_t=1000);
	    
	    	//Setters ********************************************************************
	   	void initialize(void);
	    
	    	virtual void interruptServiceRoutine(void);
		virtual void enable(void);
		virtual void disable(void);
		virtual void clear(void);
	    
	    	//Getters ********************************************************************
		uint8_t getThrottleChannel();
		uint8_t getPitchChannel();
		uint8_t getRollChannel();
		uint8_t getYawChannel();
	
	private:
		//Register *******************************************************************		
		volatile uint8_t * _pin;
		volatile uint8_t * _pcmsk;
	
		uint8_t _pcie;
		uint8_t _pcint;
		uint8_t _ch1;
		uint8_t _ch2;
		uint8_t _ch3;
		uint8_t _ch4;
		
		uint8_t _periodMicroseconds;
		static float _maxRxCycle;							// Maximum timerticks range.
	  	static float _minRxCycle;							// Minimum timerticks range.
		
		int8_t _offsetCh1;
		int8_t _offsetCh2;
		int8_t _offsetCh3;
		int8_t _offsetCh4;
		
		uint32_t _channel1;								// Double word for storing the actual timer value.
		uint32_t _channel2;
		uint32_t _channel3;
		uint32_t _channel4;
		
		uint8_t _lastChannel;
		
		float rxc2dc();
	
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
