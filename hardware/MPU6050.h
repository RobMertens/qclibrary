#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <Wire.h>

class MPU6050
{
    
  	public:
  		/* Declarations *******************************************************/
  		
		/* Constructors *******************************************************/
		MPU6050(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

		/* Setters ************************************************************/
		void initialize();
		void calibrate();
		
		void setBias(vector*); 
		void setGyroscope(vector *, vector *); 
		void setAccelero(vector *);
		void setQuaternion(quaternion *, vector *, vector *, float);
		
		/* Getters ************************************************************/
		static uint8_t getAddress();
		bool setScaleGyroscope(uint8_t);
		bool setScaleAccelero(uint8_t);
		bool getMovement(vector *, float);
		
  	private:
  		/* Declarations *******************************************************/
  		
		static uint8_t _address;
		static uint8_t _gyroscopeScale;
		static uint8_t _acceleroScale;
		
		static uint8_t _pwrMgmtRegister;
		static uint8_t _gyroscopeScaleRegister;
		static uint8_t _acceleroScaleRegister;
		static uint8_t _gyroscopeDataRegister;
		static uint8_t _acceleroDataRegister;
		
};
#endif

