#ifndef _ESC_H_
#define _ESC_H_

#include <timer16.h>

class ESC
{
	public:
		//Constructors ***************************************************************
		ESC(volatile uint8_t *, uint8_t, uint16_t=4000, uint16_t=2000, uint16_t=1000);

		//Setters ********************************************************************
		void arm();
		void writeSpeed(float, float, float, float);
		void writeMaxSpeed();
		void writeMinSpeed();
							  
		//Getters ********************************************************************
  	private:
	  	volatile uint8_t * _ddr;			// Data Direction Register (e.g.: DDRC 0b00001111).
	  	
		timer16 _t1;					// 16-bit timer.
		timer16 _t3;					// 16-bit timer.
	  	
	  	static float _maxEscCycle;			// Maximum timerticks range.
	  	static float _minEscCycle;			// Minimum timerticks range.
		
  		//Getters ********************************************************************	
    		float dc2Escc(float)
};
#endif

