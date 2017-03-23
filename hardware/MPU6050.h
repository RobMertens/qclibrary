#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdint.h>
#include <Wire.h>

class MPU6050
{
    
  	public:
  		/* Declarations *******************************************************/
  		
		/* Constructors *******************************************************/
		MPU6050(uint8_t);
		MPU6050(uint8_t, uint8_t, uint8_t);

		/* Setters ************************************************************/
		void initialize(void);
		void calibrate(void);
		
		void setBias(vector*); 
		void setRawGyroscope(vector*, vector*); 
		void setRawAccelero(vector*);
		void setQuaternion(quaternion*, vector*, vector*, float);
		
		/* Getters ************************************************************/
		uint8_t getAddress(void);
		bool setScaleGyroscope(void);
		bool setScaleAccelero(void);
		bool getMovement(vector*, float);
		
  	private:
  		/* Declarations *******************************************************/
  		
		uint8_t _address;
		uint8_t _scaleGyroscope;
		uint8_t _scaleAccelero;
		
};
#endif

