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

#include "hardware/MPU6050.hpp"

namespace qc
{

namespace component
{

using namespace math_helpers;

/** Constructors/destructors/overloads ****************************************/
MPU6050::MPU6050(const uint8_t address) : b_(Vector())
{
	address_ = address;
}

/*MPU6050::~MPU6050(void)
{
	delete b_;
}*/

/** Settings ******************************************************************/
void MPU6050::initialize(const uint8_t gyroscopeScale,
	const uint8_t acceleroScale)
{
	//Start I2C_.
	I2C_.begin();
	I2C_.setClock(400000L);

	//Start MPU
	I2C_.beginTransmission(address_);
	I2C_.write(pwrmgmt_);
	I2C_.write(0x00);
	I2C_.endTransmission();

	//Set scale.
	if(setGyroscopeScale(gyroscopeScale)==0)
	{
		gyroscopeScale_ = gyroscopeScale;
	}
	if(setAcceleroScale(acceleroScale)==0)
	{
		acceleroScale_  = acceleroScale;
	}
}

int8_t MPU6050::setGyroscopeScale(const uint8_t scale)
{
	int8_t ret = -1;

	// Set gyroscope to 500 degrees per second scale.
	I2C_.beginTransmission(address_);
	I2C_.write(gyrcnfg_);
	I2C_.write(scale);
	I2C_.endTransmission();

	// Perform check on gyroscope scaling.
	// If register setting is set correctly set success bit.
	I2C_.beginTransmission(address_);
	I2C_.write(gyrcnfg_);
	I2C_.endTransmission();
	I2C_.requestFrom(address_, (uint8_t)1);
	while(I2C_.available() < 1);
    		if(I2C_.read()==scale)
		{
			gyroscopeScale_ = scale;
			ret = 0;
		}

	return ret;
}

int8_t MPU6050::setAcceleroScale(const uint8_t scale)
{
	int8_t ret = -1;

	// Set gyroscope to xg degrees per second scale.
	I2C_.beginTransmission(address_);
	I2C_.write(acccnfg_);
	I2C_.write(scale);
	I2C_.endTransmission();

	// Perform check on accelero scaling.
	// If register setting is set correctly set success bit.
	I2C_.beginTransmission(address_);
	I2C_.write(acccnfg_);
	I2C_.endTransmission();
	I2C_.requestFrom(address_, (uint8_t)1);
	while(I2C_.available() < 1);
		if(I2C_.read()==scale)
		{
			acceleroScale_ = scale;
			ret = 0;
		}

	return ret;
}

/** Runtime functions *********************************************************/
void MPU6050::updateGyroscopeBias(void)
{
	//Local variables.
	int16_t packet[3];

	//Read the MPU-6050 gyroscope data registers.
	I2C_.beginTransmission(address_);
	I2C_.write(gyrdata_);
	I2C_.endTransmission();
	I2C_.requestFrom(address_, (uint8_t)6);
	while(I2C_.available() < 6);
	packet[0] = I2C_.read()<<8|I2C_.read();
	packet[1] = I2C_.read()<<8|I2C_.read();
	packet[2] = I2C_.read()<<8|I2C_.read();

	//Store in global vector.
	b_.x((double)( packet[0])/(gyroConversionRate_));
	b_.y((double)( packet[1])/(gyroConversionRate_));
	b_.z((double)(-packet[2])/(gyroConversionRate_));
}

void MPU6050::updateGyroscopeBias(Vector& b)
{
	//Local variables.
	int16_t packet[3];

	//Read the MPU-6050 gyroscope data registers.
	I2C_.beginTransmission(address_);
	I2C_.write(gyrdata_);
	I2C_.endTransmission();
	I2C_.requestFrom(address_, (uint8_t)6);
	while(I2C_.available() < 6);
	packet[0] = I2C_.read()<<8|I2C_.read();
	packet[1] = I2C_.read()<<8|I2C_.read();
	packet[2] = I2C_.read()<<8|I2C_.read();

	//Store in global vector.
	b.x((double)( packet[0])/(gyroConversionRate_));
	b.y((double)( packet[1])/(gyroConversionRate_));
	b.z((double)(-packet[2])/(gyroConversionRate_));
}

void MPU6050::updateGyroscope(Vector& w)
{
	//Local variables.
	int16_t packet[3];

	//Read the MPU-6050 gyroscope data registers.
	I2C_.beginTransmission(address_);
	I2C_.write(gyrdata_);
	I2C_.endTransmission();
	I2C_.requestFrom(address_, (uint8_t)6);
	while(I2C_.available() < 6);
	packet[0] = I2C_.read()<<8|I2C_.read();
	packet[1] = I2C_.read()<<8|I2C_.read();
	packet[2] = I2C_.read()<<8|I2C_.read();

	//Store in global vector.
	//Flip the z-axis.
	w.x((double)( packet[0])/(gyroConversionRate_));
	w.y((double)( packet[1])/(gyroConversionRate_));
	w.z((double)(-packet[2])/(gyroConversionRate_));

	//Subtract the bias vector.
	w -= b_;
}

void MPU6050::updateGyroscope(const Vector& b, Vector& w)
{
	//Local variables.
	int16_t packet[3];

	//Read the MPU-6050 gyroscope data registers.
	I2C_.beginTransmission(address_);
	I2C_.write(gyrdata_);
	I2C_.endTransmission();
	I2C_.requestFrom(address_, (uint8_t)6);
	while(I2C_.available() < 6);
	packet[0] = I2C_.read()<<8|I2C_.read();
	packet[1] = I2C_.read()<<8|I2C_.read();
	packet[2] = I2C_.read()<<8|I2C_.read();

	//Store in global vector.
	//Flip the z-axis.
	w.x((double)( packet[0])/(gyroConversionRate_));
	w.y((double)( packet[1])/(gyroConversionRate_));
	w.z((double)(-packet[2])/(gyroConversionRate_));

	//Subtract the bias vector.
	w -= b;
}

void MPU6050::updateAccelero(Vector& a)
{
	//Local variables.
	int16_t packet[3];

	//Read the MPU-6050 accelerometer data registers.
	I2C_.beginTransmission(address_);
	I2C_.write(accdata_);
	I2C_.endTransmission();
	I2C_.requestFrom(address_, (uint8_t)6);
	while(I2C_.available() < 6);
	packet[0] = I2C_.read()<<8|I2C_.read();
	packet[1] = I2C_.read()<<8|I2C_.read();
	packet[2] = I2C_.read()<<8|I2C_.read();

	//Store in global vector.
	a.x((double)( packet[0])/(accConversionRate_));
	a.y((double)( packet[1])/(accConversionRate_));
	a.z((double)(-packet[2])/(accConversionRate_));
}

uint8_t MPU6050::getAddress(void)
{
	return address_;
}

uint8_t MPU6050::getGyroscopeScale(void)
{
	return gyroscopeScale_;
}

uint8_t MPU6050::getAcceleroScale(void)
{
	return acceleroScale_;
}

Vector MPU6050::getGyroscopeBias(void)
{
	return b_;
}

}; //End namespace component.

}; //End namespace qc.
