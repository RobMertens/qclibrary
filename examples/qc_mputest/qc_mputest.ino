/**
 * qc_mputest.ino
 * 
 * This file let's you test the MPU6050.
 * Remove the brackets to read in the wanted sensor data within loop.
 * 
 * TODO::use build-in class functions to reduce file size.
 * TODO::use <definitions.h>.
 * 
 * @author:	Rob Mertens
 * @date:	14/04/2017
 * @version:	0.0.0
 */

/**
 * INCLUDES
 */
//LIBRARIES
#include <definitions.h>
//#include <qclibrary.h>
#include <Wire.h>
#include <cores/math.h>

/**
 * DECLARATIONS
 */
//Raw vectors from MPU6050 measurement.
//index m : measurement
vector mOmega;
vector mAccel;
vector mBias;

//Body vectors w/ removed bias and noise.
//index b : body
vector bOmega;
vector bAccel;

//Rotation quaternions.
//index r : rotation
quaternion rQ;
vector rEuler; //RPY angles.
vector rTheta; //True angles w.r.t. Euler by integrating factors.

//World vectors.
//index w : world
vector wOmega;
vector wPosit;
vector wVeloc;
vector wAccel;

//Timers.
//index t : timer
unsigned long tWatchdog;
uint16_t tLoop;
uint16_t tEsc;
uint16_t tMpu;

//Math.
const float us = 0.000001f;
const float pi = 3.141592635898f;
const float g  = 9.81f;

//Velocity approximation.
const float dm = 0.5; //[kg]

vector dDrag;
vector dVeloc;

/**
 * SETUP
 */
void setup()
{
	//Serial init.
	Serial.begin(115200);

	//I2C init.
	Wire.begin();
	Wire.setClock(400000L);

	initialize();

	setBias(&mBias); //Assuming no movement.

	//I/O init.
	DDRB |= 0x80; //LED
}

/**
 * LOOP
 */
void loop()
{
	tWatchdog = micros();

	//Get raw data vectors from measurements.
	//Update gyroscope bias and remove accelerometer noise.
	setAcceleration(&mAccel);
	bAccel = bAccel.multiply(0.9f).sum(mAccel.multiply(0.1f));
	//if (!getMovement(&bAccel, 1.0))setBias(&mBias); //Update bias if not moving.
	setOmega(&mOmega, &mBias);
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
	setQuaternion(&rQ, &bOmega, &bAccel, tLoop);
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
	wAccel = (rQ.conj()).rotate(bAccel);wAccel.z -= 0.91f; // Remove gravity;
	wVeloc = wVeloc.sum(wAccel.multiply(g*us*tLoop)); // This is bad.

	Serial.print(wAccel.x);
	Serial.print("\t");
	Serial.print(wAccel.y);
	Serial.print("\t");
	Serial.println(wAccel.z);

	//Serial.println(tl);

	while(tLoop <= 4000)
	{
		tLoop = micros() - tWatchdog;
	}
}

/**
 * GLOBAL FUNCTIONS
 *
 * TODO::replace these by classes.
 */

/**
 * Obtain rotational velocity from MPU6050 gyroscope [RAD/S].
 */
void setOmega(vector *w, vector *bias)
{
	//Local variables.
	int16_t packet[3];

	//Read the MPU-6050 gyroscope data registers.
	Wire.beginTransmission(0x68);
	Wire.write(0x43);
	Wire.endTransmission();
	Wire.requestFrom(0x68, 6);
	while(Wire.available() < 6);
	packet[0] = Wire.read()<<8|Wire.read();
	packet[1] = Wire.read()<<8|Wire.read();
	packet[2] = Wire.read()<<8|Wire.read();

	//Store in global vector.
	w -> x = (float)(packet[0])/(3754.94f) - (bias -> x);
	w -> y = (float)(packet[1])/(3754.94f) - (bias -> y);
	w -> z = (float)(-packet[2])/(3754.94f) - (bias -> z);
	w -> m = sqrt((w -> x)*(w -> x) + (w -> y)*(w -> y) + (w -> z)*(w -> z));
}

/**
 * Obtain rotational velocity bias error from MPU6050 gyroscope [RAD/S].
 */
void setBias(vector *b)
{
	//Local variables.
	int16_t packet[3];

	//Read the MPU-6050 gyroscope data registers.
	Wire.beginTransmission(0x68);
	Wire.write(0x43);
	Wire.endTransmission();
	Wire.requestFrom(0x68, 6);
	while(Wire.available() < 6);
	packet[0] = Wire.read()<<8|Wire.read();
	packet[1] = Wire.read()<<8|Wire.read();
	packet[2] = Wire.read()<<8|Wire.read();

	//Store in global vector.
	b -> x = (float)(packet[0])/(3754.94f);
	b -> y = (float)(packet[1])/(3754.94f);
	b -> z = (float)(-packet[2])/(3754.94f);
	b -> m = sqrt((b -> x)*(b -> x) + (b -> y)*(b -> y) + (b -> z)*(b -> z));
}

/**
 * Obtain translational acceleration from MPU6050 accelerometer [G-FORCE].
 */
void setAcceleration(vector *a)
{
	//Local variables.
	int16_t packet[3];

	//Read the MPU-6050 accelerometer data registers.
	Wire.beginTransmission(0x68);
	Wire.write(0x3B);
	Wire.endTransmission();
	Wire.requestFrom(0x68, 6);
	while(Wire.available() < 6);
	packet[0] = Wire.read()<<8|Wire.read();
	packet[1] = Wire.read()<<8|Wire.read();
	packet[2] = Wire.read()<<8|Wire.read();

	//Store in global vector.
	a -> x = (float)(-packet[0])/(16384.0f);
	a -> y = (float)(-packet[1])/(16384.0f);
	a -> z = (float)(packet[2])/(16384.0f);
	a -> m = sqrt((a -> x)*(a -> x) + (a -> y)*(a -> y) + (a -> z)*(a -> z));
}

/**
 * Initialize the MPU6050 sensor.
 */
void initialize()
{
	// Start MPU
	Wire.beginTransmission(0x68);
	Wire.write(0x6B);
	Wire.write(0x00);
	Wire.endTransmission();

	delayMicroseconds(50);

	// Set gyroscope to 500 degrees per second scale.
	Wire.beginTransmission(0x68);
	Wire.write(0x1B);
	Wire.write(0x08);
	Wire.endTransmission();

	// Perform check on gyroscope scaling.
	// If register setting is wrong: EXIT + LED.
	Wire.beginTransmission(0x68);
	Wire.write(0x1B);
	Wire.endTransmission();
	Wire.requestFrom(0x68, 1);
	while(Wire.available() < 1);
	if(Wire.read() != 0x08)
	{
	PORTB |= 0x80;
	while(1);
	}

	// Set gyroscope to 2g degrees per second scale.
	Wire.beginTransmission(0x68);
	Wire.write(0x1C);
	Wire.write(0x00);
	Wire.endTransmission();

	// Perform check on accelero scaling.
	// If register setting is wrong: EXIT + LED.
	Wire.beginTransmission(0x68);
	Wire.write(0x1C);
	Wire.endTransmission();
	Wire.requestFrom(0x68, 1);
	while(Wire.available() < 1);
	if(Wire.read() != 0x00)
	{
	PORTB |= 0x80;
	while(1);
	}
}

/**
 * Calculate the attitude quaternion.
 */
void setQuaternion(quaternion *q, vector *w, vector *a, uint16_t t)
{
	//Local variable declaration.
	float alpha; //Linear interpolation complementary filter weight.

	vector g; //Gavity prediction.
	quaternion qI = quaternion(); //Unit quaternion.
	quaternion qW = quaternion(); //Integrator quaternion corresponding to rotational velocity.
	quaternion qA = quaternion(); //Correcting quaternion corresponding to acceleration.

	//Quaternion corresponding to omega.
	//Integration of quaternion.
	qW = q -> multiply(quaternion(pi, *w));
	*q = q -> sum(qW.multiply(0.5*us*(float)(t)));
	q -> norm();

	//Quaternion corresponding to acceleration.
	//Tilt correction.
	alpha = (float)(!getMovement(&*a, 1.02));
	g  = (q -> conj()).rotate(*a); // PREDICTED GRAVITY (Body2World)
	qA = quaternion(sqrt(0.5*(g.z + 1)), -g.y/sqrt(2*(g.z + 1)), g.x/sqrt(2*(g.z + 1)), 0.0f);
	qA = (qI.multiply(1 - alpha)).sum(qA.multiply(alpha)); // LERP
	qA.norm();

	// Total quaternion.
	*q = q -> multiply(qA);
}

/**
 * Get quadcopter movement status based on acceleration (raw/body/world).
 */
bool getMovement(vector *a, float p)
{
	//Local variables.
	bool movement;
	float eta;

	//If acc/g-ratio equals greater than p[%]: dynamic flight.
	//Else: take-off/landing position or hoovering.
	//Sensitivity depends on p, different for bias/quaternion.
	//GRAVITY VALUE DEPENDENT ON SENSOR.
	movement = 0;
	eta = abs((a -> m)/0.91f);
	if (eta >= p)movement = 1;

	//Return movement status.
	return movement;
}
