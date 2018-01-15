/******************************************************************************
   Quadcopter-Library-v1
   qc_mpu_test.ino

   This is a test file for the mpu6050.

   @author:	Rob Mertens
   @date:	10/05/2017
   @version:	1.1.1
 ******************************************************************************/

//Include standard headers.
#include <stdio.h>
#include <string.h>
//#include <math.h>

//Include timer headers.
#include <timer8.hpp>

//Include qclibrary headers.
#include "math_helpers.hpp"
#include "logger.hpp"
#include "hardware/MPU6050.hpp"

//Use library namespaces.
using namespace qc;
using namespace component;
using namespace math_helpers;

/**
   @brief
*/
int main()
{
  //Serial.
	logger->setBaudRate(9600);
  logger->start();

  //
  Vector meas_w_bias;
  Vector meas_w_raw;
  Vector meas_a_raw;

  //Timer.
  avr::Timer::Ptr t2_ptr(new avr::Timer8(avr::t_alias::T2));
  t2_ptr->initialize(avr::t_mode::NORMAL, avr::t_interrupt::OVF);
  t2_ptr->setPrescaler(64);
  t2_ptr->reset();

  //IMU.
  MPU6050::Ptr imu_ptr(new MPU6050(0x68));
  imu_ptr->initialize(0x08, 0x00);
  imu_ptr->updateGyroscopeBias(meas_w_bias);

  for (;;)
  {
		//
    imu_ptr->updateGyroscope(meas_w_bias, meas_w_raw);
		logger->log(Logger::Info,	meas_w_raw);

		//
    imu_ptr->updateAccelero(meas_a_raw);
		logger->log(Logger::Info,	meas_a_raw);
	}

  return 0;
}

/**
 *@brief Helper functions.
 */
struct Foo
{
  /** Friend overload magic ***************************************************/
  friend
  char operator<<(const Foo& foo, const char& c)
  {
    char ret;
    ret = strcat( reinterpret_cast<const char*>(&foo), c );
    return ret;
  }
};
