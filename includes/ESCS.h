#ifndef _ESCS_H_
#define _ESCS_H_

#include <cores/settings.h>
#include <timer16.h>

class ESCS
{
	public:
		//Constructors ***************************************************************
		ESCS(t_alias, t_alias, uint16_t=4000, uint16_t=2000, uint16_t=1000);

		//Setters ********************************************************************
		void arm(uint16_t=0x0001, uint16_t=0xF9FF);
		void writeSpeed1(float);
		void writeSpeed2(float);
		void writeSpeed3(float);
		void writeSpeed4(float);
		void writeSpeed(float, float, float, float);
		void writeMaxSpeed();
		void writeMinSpeed();
							  
		//Getters ********************************************************************
		int8_t assign1(t_alias, t_channel);
		int8_t assign2(t_alias, t_channel);
		int8_t assign3(t_alias, t_channel);
		int8_t assign4(t_alias, t_channel);
  	private:
		volatile uint8_t * _ddrx;			// Data Direction Register (e.g.: DDRC 0b00001111).
		volatile uint8_t * _ddry;
	  	
		timer16 * _tx;					// 16-bit timer.
		timer16 * _ty;					// 16-bit timer.
		
		void *setDutyCycle1(float);
		void *setDutyCycle2(float);
		void *setDutyCycle3(float);
		void *setDutyCycle4(float);
	  	
	  	static float _maxEscCycle;			// Maximum timerticks range.
	  	static float _minEscCycle;			// Minimum timerticks range.
		
  		//Getters ********************************************************************	
    		float dc2Escc(float)
};
#endif

