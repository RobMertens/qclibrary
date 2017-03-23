#ifndef _TIMER_H_
#define _TIMER_H_

extern "C" void TIMER0_OVF_vect(void) __attribute__ ((signal));
extern "C" void TIMER1_OVF_vect(void) __attribute__ ((signal));
extern "C" void TIMER2_OVF_vect(void) __attribute__ ((signal));

class timer : public interrupt::handler
{
	public:
		//Constructors ***************************************************************
		timer(void);
		timer(uint8_t, uint16_t);
		
		//Setters ********************************************************************
		static void set(uint16_t);
		static void reset(void);
		static void setPrescaler(volatile uint8_t);
		
		virtual void interruptServiceRoutine(void);
		virtual void enable(void);
		virtual void disable(void);
		virtual void clear(void);
		
		friend void TIMER0_OVF_vect(void);
		#if defined(TIMER1_OVF_vect)
		friend void TIMER1_OVF_vect(void);
		#endif
		#if defined(TIMER2_OVF_vect)
		friend void TIMER2_OVF_vect(void);
		#endif
		
		//Getters ********************************************************************
		uint16_t getCount(void);
		uint16_t getOverflow(void);
		
		
	private:
		
		uint16_t _overflow;
		
		volatile uint8_t * _pin;
		volatile uint8_t * _pcmsk;
		
		volatile uint8_t *_tccr;			//PRESCALER
		volatile uint16_t *_tcnt;			//TIMER COUNT
		
		
};
#endif