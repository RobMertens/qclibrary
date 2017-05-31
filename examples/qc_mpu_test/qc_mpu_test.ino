/******************************************************************************
 * Quadcopter-Library-v1
 * qc_mpu_test.ino
 * 
 * This is a test file for the mpu6050.
 *
 * @author:	Rob Mertens
 * @date:	10/05/2017
 * @version:	1.1.1
 ******************************************************************************/

#include <math.h>
#include "timer8.h"
#include "vmath.h"
#include "hardware/MPU6050.h"

/******************************************************************************
 * DECLARATIONS
 ******************************************************************************/
float pMove = 1.02;						//2% difference in gravity vector means movement.

vector mOmega;							//Raw vectors from MPU6050 measurement.
vector mAccel;							//index m : measurement

vector bOmega;							//Body vectors w/ removed bias and noise.
vector bAccel;							//index b : body

quaternion rQ;							//Rotation quaternions.
								//index r : rotation
vector rEuler; 							//RPY angles.
vector rTheta; 							//True angles w.r.t. Euler by integrating factors.

vector wAccel;							//World vectors.
vector wAccelGravComp;

timer8 t(t_alias::T2);

MPU6050 imu(0x68);						//MPU6050.

/******************************************************************************
 * SETUP
 ******************************************************************************/
void setup()
{
	//Serial.
	Serial.begin(9600);
	
	//Timer.
	t.initialize(t_mode::NORMAL, t_interrupt::OVF);
	t.setPrescaler(64);
	
	//IMU.
	imu.initialize(0x08, 0x00);
	imu.setGyroscopeBias();
	
}

/******************************************************************************
 * LOOP
 ******************************************************************************/
void loop()
{
	//Get raw data vectors from measurements.
	if(!getMovement(pMove))
	{
		imu.setGyroscopeBias();
	}
	imu.updateGyroscope(&mOmega);
	imu.updateAccelero(&mAccel);
	
	//printRawVectors();
	
	//Update local vars.
	bOmega = mOmega;
	bAccel = bAccel.multiply(0.9f).sum(mAccel.multiply(0.1f));
	
	//printBodyVectors();
	
	//Update rotation quaternion.
	updateActualAttitude(&rQ, &bOmega, &bAccel);
	
	printQuaternion();
	
	//Update world vectors.
	wAccel = (rQ.conj()).rotate(bAccel);
	wAccelGravComp = wAccel;
	wAccelGravComp.z -= 0.91f;
	
	//printWorldAccelVector();
	//printWorldAccelGravCompVector();
}

/******************************************************************************
 * EXTERNAL FUNCTIONS
 * These functions belong in the controller class but cannot be used here.
 * Other functions are used for printing w/ serial.
 ******************************************************************************/
void updateActualAttitude(quaternion * q, vector * w, vector * a)
{
	//Local variable declaration.
	float alpha;								//Linear interpolation complementary filter weight.
	float time;
	vector worldGravEst;							//Gavity prediction.
	quaternion qDiff;
	quaternion qEst;
	quaternion qCorr;
	quaternion qI = quaternion();							
	
	//Time calc.
	time = t.getNonResetCount()*0.0625f*64.0f*0.000001f;
	t.reset();
	
	//Gyroscope integration estimation.
	qDiff = q->cross(quaternion(3.1415f, *w));
	qEst  = q->sum(qDiff.multiply(0.5*time));				//Integrate rotational velocity with looptime.
	qEst.norm();
	
	//Accelero gravity vector correction.
	if(!getMovement(pMove))							// 2% determined by testing.
	{
		alpha=1.0f;
	}
	else
	{
		alpha=0.0f;
	}
	worldGravEst  = (q->conj()).rotate(*a);				// PREDICTED GRAVITY (Local2World)
	qCorr = quaternion( sqrt(0.5*(worldGravEst.z + 1.0)		  ),
			   -worldGravEst.y/sqrt(2.0*(worldGravEst.z + 1.0)),
			    worldGravEst.x/sqrt(2.0*(worldGravEst.z + 1.0)),
			    0.0f					  );
	qCorr = (qI.multiply(1 - alpha)).sum(qCorr.multiply(alpha));		// LERP
	qCorr.norm();								// Corrected quaternion.
	
	//Magnetic correction for yaw-angle.
	//Not available on MPU6050.
	//Extra sensor.
	
	//Total quaternion.
	*q = qEst.cross(qCorr);
	
	//Euler representation.
	rEuler = q->q2euler();
}

bool getMovement(float p)
{
	//Local variables.
	bool movement;
	float eta;
	
	//If acc/g-ratio equals greater than p[%]: dynamic flight.
	//Else: take-off/landing position or hoovering.
	//Sensitivity depends on p, different for bias/quaternion.
	//GRAVITY VALUE DEPENDENT ON SENSOR.
	movement = false;
	eta = abs((bAccel.m)/0.91f);					//TODO::determine gravity vector at stand-still, like gyro bias.
	if(eta >= p)
	{	
		movement = true;
	}
	
	//Return movement status.
	return movement;
}

void printRawVectors(void)
{
	Serial.print(mOmega.x);
	Serial.print("\t");
	Serial.print(mOmega.y);
	Serial.print("\t");
	Serial.print(mOmega.z);
	Serial.print("\t");
	Serial.print(mAccel.x);
	Serial.print("\t");
	Serial.print(mAccel.y);
	Serial.print("\t");
	Serial.println(mAccel.z);
}

void printBodyVectors(void)
{
	Serial.print(bOmega.x);
	Serial.print("\t");
	Serial.print(bOmega.y);
	Serial.print("\t");
	Serial.print(bOmega.z);
	Serial.print("\t");
	Serial.print(bAccel.x);
	Serial.print("\t");
	Serial.print(bAccel.y);
	Serial.print("\t");
	Serial.println(bAccel.z);
}

void printQuaternion(void)
{
	Serial.print(rQ.w);
	Serial.print("\t");
	Serial.print(rQ.x);
	Serial.print("\t");
	Serial.print(rQ.y);
	Serial.print("\t");
	Serial.println(rQ.z);
}

void printWorldAccelVector(void)
{
	Serial.print(wAccel.x);
	Serial.print("\t");
	Serial.print(wAccel.y);
	Serial.print("\t");
	Serial.println(wAccel.z);
}

void printWorldAccelGravCompVector(void)
{
	Serial.print(wAccel.x);
	Serial.print("\t");
	Serial.print(wAccel.y);
	Serial.print("\t");
	Serial.println(wAccel.z);
}
