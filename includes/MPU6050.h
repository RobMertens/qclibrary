#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <Wire.h>
#include <cores/math.h>

class MPU6050
{
    
  	public:
  		/* Declarations *******************************************************/
  		
		/* Constructors *******************************************************/
		MPU6050(uint8_t, uint8_t=0x00, uint8_t=0x00, uint8_t=0x1B, uint8_t=0x1C, uint8_t=0x43, uint8_t=0x3B);

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
	
	class data
	{
		public:
			vector mOmega;						//Raw vectors from MPU6050 measurement.
			vector mAccel;						//m index stands for _m_easured
			vector mBias;
			
			vector bOmega;						//Body vectors w/ removed bias and noise.
			vector bAccel;						//b index stands for _b_ody
			
			quaternion rQ;						//Rotation quaternions.
			vector rEuler;						//r index stands for _r_otation
			vector rTheta;						//True angles w.r.t. Euler by integrating factors.
										//RPY angles.
			vector wOmega;						//World vectors.
			vector wPosit;						//These vectors contain the most usefull information about our quadcopter.
										//The YAW angle measurement cannot be corrected because I don't have a magnetometer.
										//This leads to poor YAW control.
										//w index stands for _w_orld
			vector wVeloc; 						//After integration. (crap)
			vector wAccel; 						//Gravity compensated.
		private:
	}
		
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

