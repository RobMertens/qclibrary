#ifndef _ESC_H_
#define _ESC_H_

#include <stdint.h>
#include <cores/timer.h>

class ESC
{
	public:
		//Constructors ***************************************************************
		ESC(volatile uint8_t *, uint8_t, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint16_t *, volatile uint8_t *, int, int);

		//Setters ********************************************************************
		void arm();
		void writeVariableSpeed(int, int, int, int);
		void writeMinimumSpeed();
		void writeMaximumSpeed();
							  
		//Getters ********************************************************************
  	private:
	  	volatile uint8_t * _ddr;			// Data Direction Register (e.g.: DDRC 0b00001111).
	  	volatile uint8_t * _pin;			// Pin Input Register for reading.
	  	volatile uint8_t * _port;			// Pin Output Register for writing.
	  	
		timer _escTimer;				// 16-bit timer.
		//volatile uint8_t * _tccr;			// Timer registers not as private variable for ESC-class.
		//volatile uint16_t * _tcnt;
		//volatile uint8_t * _timsk;
		
	  	uint8_t _esc1;					// Byte containing OUTPUT esc1 (e.g.: 0b00000001).
	  	uint8_t _esc2;					// Byte containing OUTPUT esc2 (e.g.: 0b00000010).
	  	uint8_t _esc3;					// Byte containing OUTPUT esc3 (e.g.: 0b00000100).
	  	uint8_t _esc4;					// Byte containing OUTPUT esc4 (e.g.: 0b00001000).
	  	
	  	static uint8_t _HIGH;				// Byte setting OUTPUT HIGH (e.g.: |= 0b00001111).
	  	static uint8_t _LOW;				// Byte setting OUTPUT LOW  (e.g.: &= 0b11110000).
	  	
	  	static int _minTicks;					// Minimum timerticks range.
	  	static int _maxTicks;					// Maximum timerticks range.
		
		static int _US2T; 		
		
  		//Getters ********************************************************************	
  		int microseconds2Ticks(int);
    
};
#endif

