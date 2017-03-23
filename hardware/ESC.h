#ifndef _ESC_H_
#define _ESC_H_

#include <stdint.h>

class ESC
{

  public:

  //Constructors ***************************************************************
  	ESC(void);
  	ESC(int, int);
    
  //Setters ********************************************************************
    void initialize();
  	void writeVariableSpeed(int, int, int, int);
    void writeMinimumSpeed();
    void writeMaximumSpeed();
						  
  //Getters ********************************************************************
  
  private:  
  	
  	volatile uint8_t * _ddr;	// Data Direction Register (ex.: DDRC 0b00001111).
  	volatile uint8_t * _pin;	// Pin Input Register for reading.
  	volatile uint8_t * _port;	// Pin Output Register for writing.
  	  
  	uint8_t _esc1;				// Byte containing OUTPUT esc1 (ex.: 0b00000001).
  	uint8_t _esc2;				// Byte containing OUTPUT esc2 (ex.: 0b00000010).
  	uint8_t _esc3;				// Byte containing OUTPUT esc3 (ex.: 0b00000100).
  	uint8_t _esc4;				// Byte containing OUTPUT esc4 (ex.: 0b00001000).
  	
  	uint8_t _HIGH;				// Byte setting OUTPUT HIGH (ex.: |= 0b00001111).
  	uint8_t _LOW;				// Byte setting OUTPUT LOW  (ex.: &= 0b11110000).
  	
  	int _minTicks;				// Minimum timerticks range.
  	int _maxTicks;				// Maximum timerticks range.
  	
  //Getters ********************************************************************	
  	int microsecondsToTicks(int);
    
};
#endif

