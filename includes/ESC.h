#ifndef _ESC_H_
#define _ESC_H_

#include <cores/settings.h>
#include <timer16.h>

class ESC
{
	public:
		//Constructors ***************************************************************
		ESC(t_alias, t_channel, uint16_t=4000, uint16_t=2000, uint16_t=1000);

		//Setters ********************************************************************
		void arm(uint16_t=0x0001, uint16_t=0xF9FF);
		void writeSpeed(float);
		void writeMaxSpeed();
		void writeMinSpeed();
							  
		//Getters ********************************************************************
  	private: 	
		timer16 * _t;					// 16-bit timer.
		
		void *setDutyCycle(float);
	  	
	  	static float _maxEscCycle;			// Maximum timerticks range.
	  	static float _minEscCycle;			// Minimum timerticks range.
		
  		//Getters ********************************************************************	
    		int8_t assign(t_channel);
		float dc2Escc(float)
};
#endif

