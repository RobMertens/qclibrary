#ifndef _ESC_H_
#define _ESC_H_

//#include <memory>
#include <stdint.h>
#include "settings.h"
#include "timer16.h"

using namespace t_settings;

class timer16;

class ESC
{
	public:
		//Typedefs *****************************************************************
		typedef ESC * ptr; //typedef std::shared_ptr<ESC> ptr;
		typedef ESC * const cptr; //std::shared_ptr<ESC const> cptr;

		//Constructors *************************************************************
		ESC(const alias, const channel, const uint16_t=4000, const uint16_t=2000, const uint16_t=1000);

		//Setters ******************************************************************
		void arm(const uint16_t=0x0001, const uint16_t=0xF9FF);
		void unarm(void);

		//Getters ******************************************************************
		int8_t writeSpeed(const float);
		int8_t writeMaxSpeed(void);
		int8_t writeMinSpeed(void);

  private:
		//Variables ****************************************************************
		timer16 _t;																																	// 16-bit timer.
	  float _maxEscCycle;																													// Maximum timerticks range.
	  float _minEscCycle;																													// Minimum timerticks range.

  	//Getters ******************************************************************
    int8_t assign(const channel);
		int8_t (timer16::*setDutyCycle)(float);
		float dc2Escc(float);

};
#endif
