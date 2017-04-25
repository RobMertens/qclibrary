#ifndef _RX_H_
#define _RX_H_

#include <timer8.h>

extern "C" void PCINT0_vect(void) __attribute__ ((signal));
extern "C" void PCINT1_vect(void) __attribute__ ((signal));
extern "C" void PCINT2_vect(void) __attribute__ ((signal));
extern "C" void PCINT3_vect(void) __attribute__ ((signal));

class RX : public interrupt::handler
{

  	public:

  	//Constructors ***************************************************************
    	RX(volatile uint8_t *, volatile uint8_t *, uint8_t, uint8_t);
    
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
		
	volatile uint8_t * _pin;
	volatile uint8_t * _pcmsk;
	
	uint8_t _pcie;
	uint8_t _pcint;
	uint8_t _ch1;
	uint8_t _ch2;
	uint8_t _ch3;
	uint8_t _ch4;
	
	volatile uint16_t _channel1;				// Word for storing the actual timer value.
	volatile uint16_t _channel2;
	volatile uint16_t _channel3;
	volatile uint16_t _channel4;
	
	static timer8 * _t2[4];
	
	volatile uint8_t _lastChannel;
	
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
