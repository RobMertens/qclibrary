#ifndef _MPU6050_H_
#define _MPU6050_H_

//#include <memory>
#include <Wire.h>
#include "vmath.h"

//TODO::sensors namespace for multiple sensors.
//namespace sensors
//{

using namespace vmath;

class MPU6050
{
  	public:
    /* Typedefs ***************************************************************/
    typedef MPU6050 * ptr; //std::shared_ptr<MPU6050> ptr;
    typedef MPU6050 * const cptr; //std::shared_ptr<MPU6050 const> cptr;

    /* Declarations ***********************************************************/

		/* Constructors ***********************************************************/
		MPU6050(const uint8_t);

		/* Setters ****************************************************************/
		void initialize(const uint8_t=0x00, const uint8_t=0x00);
		void setGyroscopeBias(void);
		void updateGyroscope(vector::cptr&);
		void updateAccelero(vector::cptr&);

		/* Getters ****************************************************************/
		int8_t setGyroscopeScale(const uint8_t);
		int8_t setAcceleroScale(const uint8_t);
		uint8_t getAddress(void);
		uint8_t getGyroscopeScale(void);
		uint8_t getAcceleroScale(void);
    vector::cptr getGyroscopeBias(void);

  	private:
  	/* Declarations ***********************************************************/
		uint8_t _address;
		uint8_t _gyroscopeScale;
		uint8_t _acceleroScale;

		//Registers.
		const static uint8_t _pwrmgmt = 0x6B;
		const static uint8_t _gyrcnfg = 0x1B;
		const static uint8_t _acccnfg = 0x1C;
		const static uint8_t _gyrdata = 0x43;
		const static uint8_t _accdata = 0x3B;

		//Gyroscope bias.
		vector _b;
}; //End MPU class.

//}; //End sensors namespace.
#endif
