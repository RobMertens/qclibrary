#ifndef _LED_H_
#define _LED_H_

class LED
{
	public:
		//Constructors ***************************************************************
		LED(volatile uint8_t *, uint8_t, volatile uint8_t *, volatile uint8_t *);

		//Setters ********************************************************************
		void setLed();
		void resetLed();
		void toggleLed();

		//Getters ********************************************************************
		bool getLedState();
  
	private:
		//Registers ******************************************************************
		volatile uint8_t * _ddr;						// Data Direction Register (ex.: DDRC 0b00001111).
		volatile uint8_t * _pin;						// Pin Input Register for reading.
		volatile uint8_t * _port;						// Pin Output Register for writing.
		
		//Variables ******************************************************************
		uint8_t _HIGH;								// Variable with HIGH LED-state.
		uint8_t _LOW;								// Variable with LOW LED-state.
};
#endif

