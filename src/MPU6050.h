#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <Wire.h>
#include "vmath.h"
#include "settings.h"
#include "timer8.h"
#include "timer16.h"

class MPU6050
{
    
  	public:
  		/* Declarations *******************************************************/
  		
		/* Constructors *******************************************************/
		MPU6050(uint8_t, t_alias=t_alias::T2, uint8_t=0x00, uint8_t=0x00);

		/* Setters ************************************************************/
		void initialize();
		void calibrate();
		
		void setGyroscopeBias(); 
		void updateGyroscope(vector *); 
		void updateAccelero(vector *);
		
		/* Getters ************************************************************/
		uint8_t getAddress();
		int8_t setGyroscopeScale(uint8_t);
		int8_t setAcceleroScale(uint8_t);
		vector getGyroscopeBias();
		
  	private:
  		/* Declarations *******************************************************/
  		
		uint8_t _address;
		uint8_t _gyroscopeScale;
		uint8_t _acceleroScale;
		
		//Registers.
		const uint8_t * _pwrmgmt;
		const uint8_t * _gyrcnfg;
		const uint8_t * _acccnfg;
		const uint8_t * _gyrdata;
		const uint8_t * _accdata;
		
		//Gyroscope bias.
		vector _b;
		
		timer8 _t;
		//timer16 _t;		
};
#endif

