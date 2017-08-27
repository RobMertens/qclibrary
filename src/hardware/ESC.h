#ifndef _ESC_H_
#define _ESC_H_

#include <stdint.h>
#include "settings.h"
#include "timer16.h"

//Forward declaration.
class timer16;

class ESC
{
	public:
		//Constructors ***************************************************************
		ESC(const t_alias, const t_channel, const uint16_t=4000, const uint16_t=2000, const uint16_t=1000);

		//Setters ********************************************************************
		void arm(const uint16_t=0x0001, const uint16_t=0xF9FF);
		void unarm(void);
		int8_t writeSpeed(float);
		int8_t writeMaxSpeed(void);
		int8_t writeMinSpeed(void);

		//Getters ********************************************************************
  	private:
		timer16 _t;					// 16-bit timer.

	  float _maxEscCycle;				// Maximum timerticks range.
	  float _minEscCycle;				// Minimum timerticks range.

  	//Getters ********************************************************************
    int8_t assign(const t_channel);
		int8_t (timer16::*setDutyCycle)(float);

		float dc2Escc(float);
};
#endif
