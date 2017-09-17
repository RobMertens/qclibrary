#ifndef _LED_H_
#define _LED_H_

//#include <memory>
#include <stdint.h>

class LED
{
	public:
		//Typedefs *****************************************************************
		typedef LED * ptr; //std::shared_ptr<LED> ptr;
		typedef LED * const cptr; //std::shared_ptr<LED const> cptr;

		//Constructors *************************************************************
		LED(const volatile uint8_t * const&, const uint8_t, const volatile uint8_t * const&, const volatile uint8_t * const&);

		//Setters ******************************************************************
		void set(void);
		void reset(void);
		void toggle(void);

		//Getters ******************************************************************
		bool getState(void);

	private:
		//Registers ****************************************************************
		volatile uint8_t * _ddr;						// Data Direction Register (ex.: DDRC 0b00001111).
		volatile uint8_t * _pin;						// Pin Input Register for reading.
		volatile uint8_t * _port;						// Pin Output Register for writing.

		//Variables ****************************************************************
		uint8_t _HIGH;								// Variable with HIGH LED-state.
		uint8_t _LOW;								// Variable with LOW LED-state.
};
#endif
