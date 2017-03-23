#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>

class LED
{

  public:

  //Constructors ***************************************************************
  	LED();
    
  //Setters ********************************************************************
  	void setLedStateHigh();
  	void setLedStateLow();
  	void invertLedState();
  	
  //Getters ********************************************************************
  	bool getLedState();
  
  private:  
  	
  	volatile uint8_t * _ddr;	// Data Direction Register (ex.: DDRC 0b00001111).
  	volatile uint8_t * _pin;	// Pin Input Register for reading.
  	volatile uint8_t * _port;	// Pin Output Register for writing.
  	
  	uint8_t _HIGH;				// Variable with HIGH LED-state.
  	uint8_t _LOW;				// Variable with LOW LED-state.
    
};
#endif

