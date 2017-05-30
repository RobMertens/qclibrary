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

#include <src/MPU6050.h>

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
	
	//TODO::do this for 8- and 16-bit timers.
	_t = timer8(alias);
	
	//TODO::one timer class, timer8 and timer16 inherent.
	/**switch(alias)
	{
		case(t_alias::T0) :
		case(t_alias::T2) :
			_t = timer8(alias);
			break;
		
		case(t_alias::T1) :
		case(t_alias::T3) :
		case(t_alias::T4) :
		case(t_alias::T5) :
			_t = timer16(alias);
			break;
		
		case(t_alias::NONE) :
		case(t_alias::TX) :
			//TODO::error.
			break;
	}*/
}

/*******************************************************************************
 * Method for activating the MPU6050.
 ******************************************************************************/
void MPU6050::initialize()
{	
	//Registers.
	*_pwrmgmt = 0x6B;
	*_gyrcnfg = 0x1B;
	*_acccnfg = 0x1C;
	*_gyrdata = 0x43;
	*_accdata = 0x3B;
	
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
	
	//Gyroscope bias init.
	_b = vector();								//Zero vector.
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
 ******************************************************************************/
void MPU6050::setGyroscopeBias()
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
	_b.x = (float)( packet[0])/(3754.94f);
	_b.y = (float)( packet[1])/(3754.94f);
	_b.z = (float)(-packet[2])/(3754.94f);
	_b.mag();
}

/*******************************************************************************
 * Method for reading the measurement data from the gyroscope.
 *
 * @param: *w The omega vector (w_x, w_y, w_z)' pointed to in [rad/s].
 * @param: b The bias vector (b_x, b_y, b_z)' in [rad/s].
 ******************************************************************************/
void MPU6050::updateGyroscope(vector *w)
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
	w -> x = (float)( packet[0])/(3754.94f) - (_b.x);
	w -> y = (float)( packet[1])/(3754.94f) - (_b.y);
	w -> z = (float)(-packet[2])/(3754.94f) - (_b.z);
	w -> mag();
}

/*******************************************************************************
 * Method for reading the measurement data from the accelerometer.
 *
 * @param: *a The raw acceleration vector (a_x, a_y, a_z)' pointed to in [g].
 ******************************************************************************/
void MPU6050::updateAccelero(vector *a)
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
 * Method for getting the address of the MPU6050.
 *
 * @return _address The address of the I2C-device.
 ******************************************************************************/
uint8_t MPU6050::getAddress()
{	
	return _address;
}

/*******************************************************************************
 * Method for getting the scale-setting of the gyroscope.
 * All possibilities are: 0x00 for +-250dps,
 *    			  0x08 for +-500dps,
 *		 	  0x10 for +-1000dps and
 *			  0x18 for +-2000dps.
 *
 * @return _gyroscopeScale The scale-setting of the gyroscope [byte].
 ******************************************************************************/
uint8_t MPU6050::getGyroscopeScale()
{	
	return _gyroscopeScale;
}

/*******************************************************************************
 * Method for getting the scale-setting of the accelero.
 * All possibilities are: 0x00 for +-2g,
 *    			  0x08 for +-4g,
 *			  0x10 for +-8g and
 *			  0x18 for +-16g.
 *
 * @return _acceleroScale The scale-setting of the accelerometer [byte].
 ******************************************************************************/
uint8_t MPU6050::getAcceleroScale()
{	
	return _acceleroScale;
}

/*******************************************************************************
 * Method for getting the gyroscope bias vector.
 *
 * @return _b The gyroscope bias vector [rad/s].
 ******************************************************************************/
vector MPU6050::getGyroscopeBias()
{	
	return _b;
}
