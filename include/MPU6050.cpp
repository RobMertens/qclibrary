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
MPU6050::MPU6050(uint8_t address, t_alias alias, uint8_t gyroscopeScale, uint8_t acceleroScale)
{	
	_address 	= address;
	_gyroscopeScale = gyroscopeScale;
	_acceleroScale  = acceleroScale;
	
	_t = timer8(alias);
}

/*******************************************************************************
 * Method for activating the MPU6050.
 ******************************************************************************/
void MPU6050::initialize()
{	
	setRegisters();
	
	//Start wire.
	Wire.begin();
	Wire.setClock(400000L);
	
	//Start MPU
	Wire.beginTransmission(_address);
	Wire.write(*_pwrmgmt);
	Wire.write(0x00);
	Wire.endTransmission();
	
	//Timer.
	_t.initialize(t_mode::NORMAL, t_interrupt::OVF);
	_t.setPrescaler(64);
	_t.reset();
}

/*******************************************************************************
 * Method for activating the MPU6050.
 ******************************************************************************/
void setRegisters(void)
{
	*_pwrmgmt = 0x6B;
	*_gyrcnfg = 0x1B;
	*_acccnfg = 0x1C;
	*_gyrdata = 0x43;
	*_accdata = 0x3B;
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
int8_t MPU6050::setGyroscopeScale(uint8_t scale)
{
	int8_t ret = -1;
	
	// Set gyroscope to 500 degrees per second scale.
	Wire.beginTransmission(_address);
	Wire.write(*_gyrcnfg);
	Wire.write(scale);
	Wire.endTransmission();
	
	// Perform check on gyroscope scaling.
	// If register setting is set correctly set success bit.
	Wire.beginTransmission(_address);
	Wire.write(*_gyrcnfg);
	Wire.endTransmission();
	Wire.requestFrom(_address, (uint8_t)1);
	while(Wire.available() < 1);
    		if(Wire.read()==scale)
		{
			_gyroscopeScale = scale;
			ret = 0;
		}
	
	return ret;
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
int8_t MPU6050::setAcceleroScale(uint8_t scale)
{
	int8_t ret = -1;
	
	// Set gyroscope to xg degrees per second scale.
	Wire.beginTransmission(_address);
	Wire.write(*_acccnfg);
	Wire.write(scale);
	Wire.endTransmission();

	// Perform check on accelero scaling.
	// If register setting is set correctly set success bit.
	Wire.beginTransmission(_address);
	Wire.write(*_acccnfg);
	Wire.endTransmission();
	Wire.requestFrom(_address, (uint8_t)1);
	while(Wire.available() < 1);
		if(Wire.read()==scale)
		{
			_acceleroScale = scale;
			ret = 0;
		}  
	
	return ret;
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
	Wire.write(*_gyrdata);
	Wire.endTransmission();
	Wire.requestFrom(_address, (uint8_t)6);
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
	Wire.write(*_gyrdata);
	Wire.endTransmission();
	Wire.requestFrom(_address, (uint8_t)6);
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
	Wire.write(*_accdata);
	Wire.endTransmission();
	Wire.requestFrom(_address, (uint8_t)6);
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
 ******************************************************************************/
void MPU6050::setQuaternion(quaternion *q, vector *w, vector *a)
{
	//Local variable declaration.
	float alpha;						//Linear interpolation complementary filter weight.
	float time;
	
	vector g;						//Gavity prediction.
	quaternion qI = quaternion();				//Unit quaternion.
	quaternion qW = quaternion();				//Integrator quaternion corresponding to rotational velocity.
	quaternion qA = quaternion();				//Correcting quaternion corresponding to acceleration.
	time = ((float)_t.getNonResetCount())*0.0625f*64.0f*0.000001f;	//TODO::time calculations in timer class.
	_t.reset();
	
	// Body vector calculations.
	// These are measurements.
	qW = q->multiply(quaternion(3.1415f, *w));
	*q = q->sum(qW.multiply(0.5*time));
	q->norm();
	
	// Quaternion calculation.
	// Gyroscope integration estimation.
	// Accelero gravity vector correction.
	alpha = (float)(!getMovement(&*a, 1.02));		// 2% determined by testing.
	g  = (q->conj()).rotate(*a); 				// PREDICTED GRAVITY (Body2World)
	qA = quaternion(sqrt(0.5*(g.z + 1.0)), -g.y/sqrt(2.0*(g.z + 1.0)), g.x/sqrt(2.0*(g.z + 1.0)), 0.0f);
	qA = (qI.multiply(1 - alpha)).sum(qA.multiply(alpha));	// LERP
	qA.norm();						// Corrected quaternion.
	
	// Total quaternion.
	*q = q->cross(qA);
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
uint8_t MPU6050::getAddress()
{	
	return _address;
}
