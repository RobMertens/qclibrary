/******************************************************************************
 * Quadcopter-Library-v1
 * MPU.cpp
 *  
 * This file contains predefined functions for the IMU of the quadcopter.
 * In particulary the IMU in my hardware setup is the MPU6050 together with
 * a levelshifter. If you are using another gyroscope these functions will not
 * work. This class uses the Arduino-I2C-Master-library.
 * 
 * This class also calculates also the state vectors and attitude quaternion.
 *
 * @author: 	Rob Mertens
 * @date:	14/08/2016
 * @version: 	1.0.1
 ******************************************************************************/

#include <MPU6050.h>

/*******************************************************************************
 * Constructor for the MPU6050-class. By making an object with this
 * constructor all local variables are set. It also implements the I2C-library
 * and sets up the wire for communication.
 *
 * @param: address The I2C device address.
 * @param: gyroscopeScale The gyroscope scale setting (DEFAULT=0x00).
 * @param: acceleroScale The accelerometer scale setting (DEFAULT=0x00).
 * @param: gyroscopeScaleRegister The gyroscope scale register.
 * @param: acceleroScaleRegister The accelerometer scale register.
 * @param: gyroscopeDataRegister The (1st) gyroscope measurement data register.
 * @param: acceleroDataRegister The (1st) accelerometer measurement data register.
 ******************************************************************************/
MPU6050::MPU6050(uint8_t address, uint8_t gyroscopeScale, uint8_t acceleroScale, uint8_t gyroscopeScaleRegister, uint8_t acceleroScaleRegister, uint8_t gyroscopeDataRegister, uint8_t acceleroDataRegister)
{	
	_address  = address;
	
	_gyroscopeScale = gyroscopeScale;
	_acceleroScale  = acceleroScale;
	
	_pwrMgmtRegister        = 0x6B;
	_gyroscopeScaleRegister = gyroscopeScaleRegister;
	_acceleroScaleRegister  = acceleroScaleRegister;
	_gyroscopeDataRegister  = gyroscopeDataRegister;
	_acceleroDataRegister   = acceleroDataRegister;
	
	Wire.begin();
	Wire.setClock(400000L);
}

/*******************************************************************************
 * Method for activating the MPU6050.
 ******************************************************************************/
void MPU6050::initialize()
{
	// Start MPU
	Wire.beginTransmission(_address);
	Wire.write(_pwrMgmtRegister);
	Wire.write(0x00);
	Wire.endTransmission();
}

/*******************************************************************************
 * Method for setting the full scale of the gyroscope.
 * All possibilities are: 0x00 for +-250dps,
 *    			  0x08 for +-500dps,
 *		 	  0x10 for +-1000dps and
 *			  0x18 for +-2000dps.
 *
 * @param: value The gyroscope scale value. (DEFAULT=0x00)
 * @return succes Is the register set properly? (TRUE/FALSE)
 ******************************************************************************/
bool MPU6050::setGyroscopeScale(uint8_t value=0x00)
{
	bool success = false;
	
	// Scale value.
	_gyroscopeScale = value;
	
	// Set gyroscope to 500 degrees per second scale.
	Wire.beginTransmission(_address);
	Wire.write(_gyroscopeScaleRegister);
	Wire.write(_gyroscopeScale);
	Wire.endTransmission();
	
	// Perform check on gyroscope scaling.
	// If register setting is set correctly set success bit.
	Wire.beginTransmission(_address);
	Wire.write(_gyroscopeScaleRegister);
	Wire.endTransmission();
	Wire.requestFrom(_address, 1);
	while(Wire.available() < 1);
    		if(Wire.read() == _gyroscopeScale){success = true;}  
	
	return success;
}

/*******************************************************************************
 * Method for setting the full scale of the accelero.
 * All possibilities are: 0x00 for +-2g,
 *    			  0x08 for +-4g,
 *			  0x10 for +-8g and
 *			  0x18 for +-16g.
 *
 * @param: value The accelerometer scale value. (DEFAULT=0x00)
 * @return: succes Is the register set properly? (TRUE/FALSE)
 ******************************************************************************/
bool MPU6050::setAcceleroScale(uint8_t value=0x00)
{
	bool success = false;
	
	// Scale value.
	_acceleroScale = value;
	
	// Set gyroscope to 2g degrees per second scale.
	Wire.beginTransmission(_address);
	Wire.write(_acceleroScaleRegister);
	Wire.write(_acceleroScale);
	Wire.endTransmission();

	// Perform check on accelero scaling.
	// If register setting is set correctly set success bit.
	Wire.beginTransmission(_address);
	Wire.write(_acceleroScaleRegister);
	Wire.endTransmission();
	Wire.requestFrom(_address, 1);
	while(Wire.available() < 1);
		if(Wire.read() == _acceleroScale){success = true;}  
	
	return success;
}

/*******************************************************************************
 * Method for obtaining the gyroscope bias.
 *
 * @param: *b The bias vector [b_x, b_y, b_z]' pointed to.
 ******************************************************************************/
void MPU6050::setBias(vector *b)
{
	//Local variables.
	int16_t packet[3];
	
	//Read the MPU-6050 gyroscope data registers.
	Wire.beginTransmission(_address);
	Wire.write(_gyroscopeDataRegister);
	Wire.endTransmission();
	Wire.requestFrom(_address, 6);
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

/*******************************************************************************
 * Method for reading the measurement data from the gyroscope.
 *
 * @param: *w The omega vector (w_x, w_y, w_z)' pointed to in [rad/s].
 * @param: b The bias vector (b_x, b_y, b_z)' in [rad/s].
 ******************************************************************************/
void MPU6050::setGyroscope(vector *w, vector b)
{	
	//Local variables.
	int16_t packet[3];
	
	//Read the MPU-6050 gyroscope data registers.
	Wire.beginTransmission(_address);
	Wire.write(_gyroscopeDataRegister);
	Wire.endTransmission();
	Wire.requestFrom(_address, 6);
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

/*******************************************************************************
 * Method for reading the measurement data from the accelerometer.
 *
 * @param: *a The raw acceleration vector (a_x, a_y, a_z)' pointed to in [g].
 ******************************************************************************/
void MPU6050::setAccelero(vector *a)
{
	//Local variables.
	int16_t packet[3];
	
	//Read the MPU-6050 accelerometer data registers.
	Wire.beginTransmission(_address);
	Wire.write(_acceleroDataRegister);
	Wire.endTransmission();
	Wire.requestFrom(_address, 6);
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

/*******************************************************************************
 * Method for determining the attitude quaternion.
 * TODO::Internal system timer instead of passing through time.
 *
 * Source: "Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs"
 *
 * @param: *q The unit rotation quaternion (q_w, q_x, q_y, q_z)' pointed to [-].
 * @param: *w The raw rotational velocity vector (w_x, w_y, w_z)' pointed to [rad/s].
 * @param: *a The raw linear acceleration vector (a_x, a_y, a_z)' pointed to [g].
 * @param: time The period for integration. (TODO::Remove this)
 ******************************************************************************/
void MPU6050::setQuaternion(quaternion *q, vector *w, vector *a, float time)
{
	//Local variable declaration.
	float alpha; //Linear interpolation complementary filter weight.
	
	vector g; //Gavity prediction.
	quaternion qI = quaternion(); //Unit quaternion.
	quaternion qW = quaternion(); //Integrator quaternion corresponding to rotational velocity.
	quaternion qA = quaternion(); //Correcting quaternion corresponding to acceleration.
		
	// Body vector calculations.
	// These are measurements.
	qW = q -> multiply(quaternion(pi, *w));
	*q = q -> sum(qW.multiply(0.5*(float)(t)));
	q -> norm();
	
	// Quaternion calculation.
	// Gyroscope integration estimation.
	// Accelero gravity vector correction.
	alpha = (float)(!getMovement(&*a, 1.02));		// 2% determined by testing.
	g  = (q -> conj()).rotate(*a); 				// PREDICTED GRAVITY (Body2World)
	qA = quaternion(sqrt(0.5*(g.z + 1.0)), -g.y/sqrt(2.0*(g.z + 1.0)), g.x/sqrt(2.0*(g.z + 1.0)), 0.0f);
	qA = (qI.multiply(1 - alpha)).sum(qA.multiply(alpha));	// LERP
	qA.norm();						// Corrected quaternion.
	
	// Total quaternion.
	*q = q -> cross(qA);
}

/*******************************************************************************
 * Method for determining if the quadcopter is flying dynamical or not.
 * The total acceleration is compared w/ gravity.
 * 
 * DO NOT USE A GRAVITY CORRECTED ACCELERATION VECTOR.
 *
 * @param: *a The body/inertial linear acceleration vector (a_x, a_y, a_z)' pointed to [g].
 * @param: p The user-defired comparision percentage, e.g.: 1,02.
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
  eta = abs((a -> m)/0.91f);
  if (eta >= p)movement = true;

  //Return movement status.
  return movement;
}

/*******************************************************************************
 * Method for getting the address of the MPU6050.
 *
 * @return _address The address of the I2C-device.
 ******************************************************************************/
static uint8_t MPU6050::getAddress()
{	
    return _address;
}
