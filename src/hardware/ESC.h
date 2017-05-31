#ifndef _ESC_H_
#define _ESC_H_

#include <stdint.h>
#include "settings.h"
#include "timer16.h"

class timer16;

class ESC
{
	public:
		//Constructors ***************************************************************
		ESC(t_alias, t_channel, uint16_t=4000, uint16_t=2000, uint16_t=1000);

		//Setters ********************************************************************
		void arm(uint16_t=0x0001, uint16_t=0xF9FF);
		void unarm();
		int8_t writeSpeed(float);
		int8_t writeMaxSpeed();
		int8_t writeMinSpeed();
							  
		//Getters ********************************************************************
  	private: 	
		timer16 _t;					// 16-bit timer.
	  	
	  	float _maxEscCycle;				// Maximum timerticks range.
	  	float _minEscCycle;				// Minimum timerticks range.
		
  		//Getters ********************************************************************	
    		int8_t assign(t_channel);
		int8_t (timer16::*setDutyCycle)(float);
		
		float dc2Escc(float);
};
#endif

