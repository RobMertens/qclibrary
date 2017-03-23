/******************************************************************************
 *	Quadcopter-Library-v1
 *  MPU.cpp
 *  
 *	This file contains predefined functions for the IMU of the quadcopter.
 *	In particulary the IMU in my hardware setup is the MPU6050 together with
 *	a levelshifter. If you are using another gyroscope these functions will not
 *	work. This class uses the Arduino-I2C-Master-library. This library
 *	works faster then the standard Wire-library allthough it contains only
 *	master functions. More information about this library can be found here:
 *	
 *	http://dsscircuits.com/articles/arduino-i2c-master-library
 *
 *	This class also calculates also the state vectors and attitude quaternion.
 *
 *  @author Rob Mertens
 *  @version 1.0.1 14/08/2016
 ******************************************************************************/

#include <MPU6050.h>

/*******************************************************************************
 *  Constructor for the MPU6050-class. By making an object with this
 *	constructor all local variables are set. It also implements the I2C-library
 *	and sets up the wire for communication.
 ******************************************************************************/
MPU6050::MPU6050(uint8_t address)
{	
	_address = address;
	_gyroScale = 0x00;
	_accScale = 0x00;
	
	Wire.begin();
	Wire.setClock(400000L);
}

/*******************************************************************************
 *  Constructor for the MPU6050-class. By making an object with this
 *	constructor all local variables are set. It also implements the I2C-library
 *	and sets up the wire for communication.
 ******************************************************************************/
MPU6050::MPU6050(uint8_t address, uint8_t gyroScale, uint8_t accScale)
{	
	_address = address;
	_gyroScale = gyroScale;
	_accScale = accScale;
	
	Wire.begin();
	Wire.setClock(400000L);
}

/*******************************************************************************
 *  Method for activating the MPU6050.
 ******************************************************************************/
void MPU6050::initialize(void)
{
	// Start MPU
	Wire.beginTransmission(_address);
	Wire.write(0x6B);
	Wire.write(0x00);
	Wire.endTransmission();
}

/*******************************************************************************
 *  Method for setting the full scale of the gyroscope.
 *	All possibilities are:	0x00 for +-250dps,
 *		    				0x08 for +-500dps,
 *					 		0x10 for +-1000dps and
 *					 		0x18 for +-2000dps.
 *
 *  @return succes Is the register set properly? (TRUE/FALSE)
 ******************************************************************************/
bool MPU6050::setGyroscopeScale(void)
{
    bool success = false;
    
	// Set gyroscope to 500 degrees per second scale.
	Wire.beginTransmission(_address);
	Wire.write(0x1B);
	Wire.write(_scaleGyroscope);
	Wire.endTransmission();
	
	// Perform check on gyroscope scaling.
	// If register setting is set correctly set success bit.
	Wire.beginTransmission(_address);
	Wire.write(0x1B);
	Wire.endTransmission();
	Wire.requestFrom(_address, 1);
	while(Wire.available() < 1);
    if(TWire.read() == _scaleGyroscope)success = true;  
    
    return success;
}

/*******************************************************************************
 *  Method for setting the full scale of the accelero.
 *	All possibilities are:	0x00 for +-2g,
 *		    				0x08 for +-4g,
 *					 		0x10 for +-8g and
 *					 		0x18 for +-16g.
 *
 *  @return succes Is the register set properly? (TRUE/FALSE)
 ******************************************************************************/
bool MPU6050::setAcceleroScale(void)
{
    bool success = false;
    
	// Set gyroscope to 2g degrees per second scale.
	Wire.beginTransmission(_address);
	Wire.write(0x1C);
	Wire.write(_scaleAccelero);
	Wire.endTransmission();
	
	// Perform check on accelero scaling.
	// If register setting is set correctly set success bit.
	Wire.beginTransmission(_address);
	Wire.write(0x1C);
	Wire.endTransmission();
	Wire.requestFrom(_address, 1);
	while(Wire.available() < 1);
    if(TWI.receive() == _scaleAccelero)success = true;  
    
    return success;
}

/*******************************************************************************
 *  Method for obtaining the gyroscope bias.
 ******************************************************************************/
void MPU6050::setBias(vector *b)
{
	//Local variables.
	int16_t packet[3];
	
	//Read the MPU-6050 gyroscope data registers.
	Wire.beginTransmission(_address);
	Wire.write(0x43);
	Wire.endTransmission();
	Wire.requestFrom(_address, 6);
	while(Wire.available() < 6);
	packet[0] = Wire.read()<<8|Wire.read();
	packet[1] = Wire.read()<<8|Wire.read();
	packet[2] = Wire.read()<<8|Wire.read();
	
	//Store in global vector.
	b -> _x = (float)(packet[0])/(3754.94f);
	b -> _y = (float)(packet[1])/(3754.94f);
	b -> _z = (float)(-packet[2])/(3754.94f);
	b -> getMagnitude();
}

/*******************************************************************************
 *	Method for reading the measurement data from the gyroscope.
 *
 *  @return _dataGyroscope Vector containing raw gyroscope readings in radians per second.
 ******************************************************************************/
void MPU6050::setRawGyroscope(vector *w, vector *b)
{	
	//Local variables.
	int16_t packet[3];
	
	//Read the MPU-6050 gyroscope data registers.
	Wire.beginTransmission(_address);
	Wire.write(0x43);
	Wire.endTransmission();
	Wire.requestFrom(_address, 6);
	while(Wire.available() < 6);
	packet[0] = Wire.read()<<8|Wire.read();
	packet[1] = Wire.read()<<8|Wire.read();

	//Store in global vector.
	w -> _x = (float)(packet[0])/(3754.94f) - (b -> _x);
	w -> _y = (float)(packet[1])/(3754.94f) - (b -> _y);
	w -> _z = (float)(-packet[2])/(3754.94f) - (b -> _z);
	w -> getMagnitude();
}

/*******************************************************************************
 *	Method for reading the measurement data from the accelerometer.
 *
 *  @return _dataAccelero Vector containing raw accelero readings in g-force.
 ******************************************************************************/
void MPU6050::setRawAccelero(vector *a)
{
	//Local variables.
	int16_t packet[3];
	
	//Read the MPU-6050 accelerometer data registers.
	Wire.beginTransmission(_address);
	Wire.write(0x3B);
	Wire.endTransmission();
	Wire.requestFrom(_address, 6);
	while(Wire.available() < 6);
	packet[0] = Wire.read()<<8|Wire.read();
	packet[1] = Wire.read()<<8|Wire.read();
	packet[2] = Wire.read()<<8|Wire.read();
	
	//Store in global vector.
	a -> _x = (float)(-packet[0])/(16384.0f);
	a -> _y = (float)(-packet[1])/(16384.0f);
	a -> _z = (float)(packet[2])/(16384.0f);
	a -> getMagnitude();
}

/*******************************************************************************
 *	Method for reading the measurement data from the accelerometer.
 *	
 *	TODO::Internal system timer instead of passing through time. FUNCTION IN OTHER CLASS...
 *
 *	Source: "Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs"
 *
 *  @return _dataAccelero Vector containing raw accelero readings in g-force.
 ******************************************************************************/
void MPU6050::setQuaternion(quaternion *q, vector *w, vector *a, float time)
{
	//Local variable declaration.
	float alpha; //Linear interpolation complementary filter weight.
	
	vector g; //Gavity prediction.
	quaternion qI = quaternion(); //Unit quaternion.
	quaternion qW; //Integrator quaternion corresponding to rotational velocity.
	quaternion qA; //Correcting quaternion corresponding to acceleration.
		
	// Body vector calculations.
	// These are measurements.
	qW = q -> multiply(quaternion(pi, *w));
	*q = q -> sum(qW.multiply(0.5*(float)(t)));
	q -> norm();
	
	// Quaternion calculation.
	// Gyroscope integration estimation.
	// Accelero gravity vector correction.
	alpha = (float)(!getMovement(&*a, 1.02));
	g  = (q -> conj()).getRotation(*a); // PREDICTED GRAVITY (Body2World)
	qA = quaternion(sqrt(0.5*(g._z + 1)), -g._y/sqrt(2*(g._z + 1)), g.x/sqrt(2*(g._z + 1)), 0.0f);
	qA = (qI.multiply(1 - alpha)).sum(qA.multiply(alpha));	// LERP
	qA.norm();	// Corrected quaternion.
	
	// Total quaternion.
	*q = q -> cross(qA);
}

/*******************************************************************************
 *  Method for getting the address of the MPU6050.
 *
 *  @return _address The address of the I2C-device.
 ******************************************************************************/
bool MPU6050::getMovement(vector *a, float p)
{
  //Local variables.
  bool movement;
  float eta;

  //If acc/g-ratio equals greater than p[%]: dynamic flight.
  //Else: take-off/landing position or hoovering.
  //Sensitivity depends on p, different for bias/quaternion.
  //GRAVITY VALUE DEPENDENT ON SENSOR.
  movement = false;
  eta = abs((a -> _m)/0.91f);
  if (eta >= p)movement = true;

  //Return movement status.
  return movement;
}

/*******************************************************************************
 *  Method for getting the address of the MPU6050.
 *
 *  @return _address The address of the I2C-device.
 ******************************************************************************/
uint8_t MPU6050::getAddress()
{	
    return _address;
}