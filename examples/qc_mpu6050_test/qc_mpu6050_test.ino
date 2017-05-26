/******************************************************************************
 * Quadcopter-Library-v1
 * qc_mpu6050_test.ino
 * 
 * This is a test file for the mpu6050.
 *
 * @author:	Rob Mertens
 * @date:	10/05/2017
 * @version:	1.1.1
 ******************************************************************************/

//#include <definitions.h>
#include "vmath.h"
#include "timer8.h"
#include "MPU6050.h"

/******************************************************************************
 * DECLARATIONS
 ******************************************************************************/
vector mOmega;							//Raw vectors from MPU6050 measurement.
vector mAccel;							//index m : measurement
vector mBias;

vector bOmega;							//Body vectors w/ removed bias and noise.
vector bAccel;							//index b : body

quaternion rQ;							//Rotation quaternions.
								//index r : rotation
vector rEuler; 							//RPY angles.
vector rTheta; 							//True angles w.r.t. Euler by integrating factors.

vector wOmega;							//World vectors.
vector wPosit;							//index w : world
vector wVeloc;
vector wAccel;

MPU6050 mpu(0x68, t_alias::T2, 0x01, 0x00);					//MPU6050.

/******************************************************************************
 * MAIN
 ******************************************************************************/
int main(void)
{
	//Initialize.
	Serial.begin(9600);
		
	mpu.initialize();
	
	//Do measurements.
	for(;;)
	{
		//Get raw data vectors from measurements.
		//Update gyroscope bias and remove accelerometer noise.
		mpu.setAccelero(&mAccel);
		bAccel = bAccel.multiply(0.9f).sum(mAccel.multiply(0.1f));
		if (!mpu.getMovement(&bAccel, 1.02))mpu.setBias(&mBias); 	//Update bias if not moving.
		mpu.setGyroscope(&mOmega, &mBias);
		/**
		Serial.print(bAccel.m);
		Serial.print("\t");
		Serial.print(bOmega.x);
		Serial.print("\t");
		Serial.print(bOmega.y);
		Serial.print("\t");
		Serial.println(bOmega.z);
		*/
		
		//Update rotation quaternion.
		mpu.setQuaternion(&rQ, &bOmega, &bAccel);
		
		/**
		Serial.print(rQ.w);
		Serial.print("\t");
		Serial.print(rQ.x);
		Serial.print("\t");
		Serial.print(rQ.y);
		Serial.print("\t");
		Serial.println(rQ.z);
		*/
		
		//Update world vectors.
		wAccel = (rQ.conj()).rotate(bAccel);
		wAccel.z -= 0.91f;						// Remove gravity;
		/**
		Serial.print(wAccel.x);
		Serial.print("\t");
		Serial.print(wAccel.y);
		Serial.print("\t");
		Serial.println(wAccel.z);
		*/
	}
}
