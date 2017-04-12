#ifndef _TIMERINTERRUPT_H_
#define _TIMERINTERRUPT_H_

#include <stdint.h>

extern "C" void TIMER0_COMPA_vect(void) __attribute__ ((signal));

class timerInterrupt : public interrupt::handler
{

	public:

	//Constructors ***************************************************************
	timerInterrupt(void);

	//Setters ********************************************************************
	void initialize(void);
    
	virtual void interruptServiceRoutine(void);
	virtual void enable(void);
	virtual void disable(void);
	virtual void clear(void);

	friend void TIMER0_COMPA_vect(void);
    
	//Getters ********************************************************************
  
	private:  
		
	volatile uint8_t * _timsk;
	volatile uint8_t * _ocr;
	volatile uint8_t * _ocie;
	volatile uint8_t * _led;
};


#endif
