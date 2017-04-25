/**
 * qc_main_standalone.ino
 * 
 * This file contains a full flight controller for an arduino based quadcopter.
 * 
 * TODO::use <definitions.h> to be compatible for different hardware setups.
 * 
 * @author:	Rob Mertens
 * @date:	14/04/2017
 * @version:	1.1.1
 */

/**
 * INCLUDES
 */
//LIBRARIES
#include <definitions.h>
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
quaternion rQ;
vector rEuler; //RPY angles.
vector rTheta; //True angles w.r.t. Euler by integrating factors.

//World vectors.
vector wOmega;
vector wPosit;
vector wVeloc;
vector wAccel;

//Timers.
uint32_t tWatchdog;
uint16_t tLoop;
uint16_t tEsc;
uint16_t tMpu;

//Math.
//TODO::Add in math class as public var.
const float us = 0.000001f;
const float pi = 3.141592635898f;
const float g  = 9.81f;

//Receiver.
uint32_t rxChan1;
uint32_t rxChan2;
uint32_t rxChan3;
uint32_t rxChan4;

uint8_t rxMem;

uint16_t rxIn1;
uint16_t rxIn2;
uint16_t rxIn3;
uint16_t rxIn4;

//Input.


//Control.

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
  DDRC |= 0x0F; //ESC
  
}

/**
 * LOOP
 */
void loop()
{
	//Start looptime.
	tWatchdog = micros();
	
	//Get raw data vectors from measurements.
	//Update gyroscope bias and remove accelerometer noise.
	setAcceleration(&mAccel);
	bAccel = bAccel.multiply(0.9f).sum(mAccel.multiply(0.1f));
	if(!getMovement(&bAccel, 1.0))setBias(&mBias); 			//Update bias if not moving.
	setOmega(&mOmega, &mBias);
	bOmega = mOmega;						//Unnecessary.
	setQuaternion(&rQ, &bOmega, &bAccel, tLoop);
	rEuler = rQ.q2euler();						//Base for simple attitude control.
	
	//
	
	while(tLoop <= 4000)tLoop = micros() - tWatchdog;
}

/**
 * GLOBAL FUNCTIONS
 *
 * These functions are replaced by the library.
 */

/**
 * Obtain rotational velocity from MPU6050 gyroscope [RAD/S].
 */
void setOmega(vector *w, vector b)
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
	w -> x = (float)( packet[0])/(3754.94f) - (b.x);
	w -> y = (float)( packet[1])/(3754.94f) - (b.y);
	w -> z = (float)(-packet[2])/(3754.94f) - (b.z);
	w -> mag();
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
	b -> x = (float)( packet[0])/(3754.94f);
	b -> y = (float)( packet[1])/(3754.94f);
	b -> z = (float)(-packet[2])/(3754.94f);
	b -> mag();
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
	a -> z = (float)( packet[2])/(16384.0f);
	a -> mag();
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
	quaternion qW; //Integrator quaternion corresponding to rotational velocity.
	quaternion qA; //Correcting quaternion corresponding to acceleration.

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

/**
 * Pin-change interrupt for measuring PWM-signals from receiver.
 */
ISR(PCINT0_vect)
{
	//Receiver channel 1.
	if((rxMem & 0x01 == 0x00) 0 && (PINB & B00000001))
	{
	rxMem |= 0x01;
	rxChan1 = micros();
	}
	else if(recChannel1 == 1 && !(PINB & B00000001)){
	rxMem &= 0xFE;
	rxIn1 = 1.3 * (micros() - rxChan1) - 600;
	}

	//Receiver channel 2.
	if((rxMem & 0x02 == 0x00) && PINB & B00000010 ){
	rxMem |= 0x02;
	rxChan2 = micros();
	}
	else if(recChannel2 == 1 && !(PINB & B00000010)){
	rxMem &= 0xFD;
	rxIn2 = 1.3 * (micros() - rxChan2) - 600;
	}

	//Receiver channel 3.
	if((rxMem & 0x04 == 0x00) && PINB & B00000100 ){
	rxMem |= 0x04;
	rxChan3 = micros();
	}
	else if(recChannel3 == 1 && !(PINB & B00000100)){
	rxMem &= 0xFB;
	rxIn3 = 1.3 * (micros() - rxChan3) - 600;
	}

	//Receiver channel 4.
	if((rxMem & 0x08 == 0x00) == 0 && PINB & B00001000 ){
	rxMem &= 0x08;
	rxChan4 = micros();
	}
	else if(recChannel4 == 1 && !(PINB & B00001000)){
	rxMem &= 0xF7;
	rxIn4 = 1.3 * (micros() - rxChan4) - 600;
	}
}

/**
 * TODO::make pointer for dynamical memory storage.
 */
vector control(vector vIn, vector vDes, float kp, float ki, float kd, float limit, float t)
{
	vector vP; // P Term.
	vector vI; // I Term.
	vector vD; // D Term.

	vector vOut; // Output.

	vector vE; // New error.
	vector vL; // Old error.

	vE = vIn.substract(vDes);
	vP = vE.multiply(kp);
	vI = (vI.sum(vE.multiply(t))).multiply(ki);
	vD = (vE.substract(vL)).multiply(kd/t);

	vOut = vP.sum(vI).sum(vD);
	if(vOut.x > limit)vOut.x = limit;
	else if(vOut.x < (-1)*limit)vOut.x = (-1)*limit;
	if(vOut.y > limit)vOut.y = limit;
	else if(vOut.y < (-1)*limit)vOut.y = (-1)*limit;
	if(vOut.z > limit)vOut.z = limit;
	else if(vOut.z < (-1)*limit)vOut.z = (-1)*limit;

	vL = vE;

	return vD;
}
