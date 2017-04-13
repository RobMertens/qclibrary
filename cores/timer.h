#ifndef _TIMER_H_
#define _TIMER_H_

#include <cores/interrupt.h>
// ATMEGA2560
// TIMER0
extern "C" void TIMER0_OVF_vect(void) __attribute__ ((signal));
extern "C" void TIMER0_COMP_vect(void) __attribute__ ((signal));

// TIMER1
extern "C" void TIMER1_OVF_vect(void) __attribute__ ((signal));
//extern "C" void TIMER1_CAPT_vect(void) __attribute__ ((signal));
extern "C" void TIMER1_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER1_COMPB_vect(void) __attribute__ ((signal));
extern "C" void TIMER1_COMPC_vect(void) __attribute__ ((signal));

// TIMER2
extern "C" void TIMER2_OVF_vect(void) __attribute__ ((signal));
extern "C" void TIMER2_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER2_COMPB_vect(void) __attribute__ ((signal));

// TIMER3
extern "C" void TIMER3_OVF_vect(void) __attribute__ ((signal));
//extern "C" void TIMER3_CAPT_vect(void) __attribute__ ((signal));
extern "C" void TIMER3_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER3_COMPB_vect(void) __attribute__ ((signal));
extern "C" void TIMER3_COMPC_vect(void) __attribute__ ((signal));

// TIMER4
extern "C" void TIMER4_OVF_vect(void) __attribute__ ((signal));
//extern "C" void TIMER4_CAPT_vect(void) __attribute__ ((signal));
extern "C" void TIMER4_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER4_COMPB_vect(void) __attribute__ ((signal));
extern "C" void TIMER4_COMPC_vect(void) __attribute__ ((signal));

// TIMER5
extern "C" void TIMER5_OVF_vect(void) __attribute__ ((signal));
//extern "C" void TIMER5_CAPT_vect(void) __attribute__ ((signal));
extern "C" void TIMER5_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER5_COMPB_vect(void) __attribute__ ((signal));
extern "C" void TIMER5_COMPC_vect(void) __attribute__ ((signal));

class timer : public interrupt::handler
{
	public:
		//Constructors ***************************************************************
		timer(volatile uint8_t *, volatile uint8_t *, volatile uint8_t *);
		timer(volatile uint8_t *, volatile uint16_t *, volatile uint8_t *);
		
		//Setters ********************************************************************
		static void initialize(uint8_t, uint8_t);
		static void set(uint16_t);
		static void reset();
		static void prescaler(uint8_t);
		
		virtual void interruptServiceRoutine(void);
		virtual void enable(void);
		virtual void disable(void);
		virtual void clear(void);
		
		// TIMER0.
		friend void TIMER0_OVF_vect(void);
		friend void TIMER0_COMP_vect(void);
		
		// TIMER1.
		#if defined(TIMER1_OVF_vect)
		friend void TIMER1_OVF_vect(void);
		friend void TIMER1_COMPA_vect(void);
		friend void TIMER1_COMPB_vect(void);
		friend void TIMER1_COMPC_vect(void);
		#endif
		
		// TIMER2.
		#if defined(TIMER2_OVF_vect)
		friend void TIMER2_OVF_vect(void);
		friend void TIMER2_COMPA_vect(void);
		friend void TIMER2_COMPB_vect(void);
		#endif
		
		// TIMER3.
		#if defined(TIMER3_OVF_vect)
		friend void TIMER3_OVF_vect(void);
		friend void TIMER3_COMPA_vect(void);
		friend void TIMER3_COMPB_vect(void);
		friend void TIMER3_COMPC_vect(void);
		#endif
		
		// TIMER4.
		#if defined(TIMER4_OVF_vect)
		friend void TIMER4_OVF_vect(void);
		friend void TIMER4_COMPA_vect(void);
		friend void TIMER4_COMPB_vect(void);
		friend void TIMER4_COMPC_vect(void);
		#endif
		
		// TIMER5.
		#if defined(TIMER5_OVF_vect)
		friend void TIMER5_OVF_vect(void);
		friend void TIMER5_COMPA_vect(void);
		friend void TIMER5_COMPB_vect(void);
		friend void TIMER5_COMPC_vect(void);
		#endif
		
		//Getters ********************************************************************
		uint16_t getCount();
		uint16_t getOverflow();
		
		
	private:
		// Vars.
		int _bitness;					// Timer bitness (8/16-bit)
		
		// Registers.
		volatile uint16_t * _tcnt;			// TIMER COUNT
		volatile uint8_t  * _tcnth;			// TIMER COUNT HIGH BYTE
		volatile uint8_t  * _tcntl;			// TIMER COUNT LOW BYTE
		volatile uint8_t  * _tccr;			// PRESCALER
		volatile uint8_t  * _timsk;			// Timer Interrupt Mask register.
		volatile uint8_t  * _tifr0;			// Timer Interrupt Flag Register for timer0, timer1, timer2.
		volatile uint8_t  * _tifr1;			// Timer Interrupt Flag Register for timer2, timer3.
		volatile uint8_t  * _tifr2;			// Timer Interrupt Flag Register for timer4, timer5.
		
		// Overflow.		
		uint16_t _interruptCount;
		uint16_t _overflowCount;			// TODO::remember number of overflows.
		uint16_t _nonResetCount;
		
		
};
#endif
