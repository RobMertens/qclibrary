#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <Wire.h>
#include "vmath.h"

class MPU6050
{
    
  	public:
  		/* Declarations *******************************************************/
  		
		/* Constructors *******************************************************/
		MPU6050(uint8_t);

		/* Setters ************************************************************/
		void initialize(uint8_t=0x00, uint8_t=0x00);
		
		void setGyroscopeBias(); 
		void updateGyroscope(vector *); 
		void updateAccelero(vector *);
		
		/* Getters ************************************************************/
		int8_t setGyroscopeScale(uint8_t);
		int8_t setAcceleroScale(uint8_t);
		
		uint8_t getAddress(void);
		uint8_t getGyroscopeScale(void);
		uint8_t getAcceleroScale(void);
		vector getGyroscopeBias(void);
		
  	private:
  		/* Declarations *******************************************************/
  		
		uint8_t _address;
		uint8_t _gyroscopeScale;
		uint8_t _acceleroScale;
		
		//Registers.
		uint8_t _pwrmgmt = 0x6B;
		uint8_t _gyrcnfg = 0x1B;
		uint8_t _acccnfg = 0x1C;
		uint8_t _gyrdata = 0x43;
		uint8_t _accdata = 0x3B;
		
		//Gyroscope bias.
		vector _b;	
};
#endif

