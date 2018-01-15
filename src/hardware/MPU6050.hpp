/******************************************************************************
 * Quadcopter-Library-v1
 * MPU6050.hpp
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

#ifndef qc_MPU6050_HPP
#define qc_MPU6050_HPP

#include <Wire.h>
#include "math_helpers.hpp"

namespace qc
{

namespace component
{

using namespace math_helpers;

/**
 * @brief
 */
class MPU6050
{
  public:
		/** Typedefs **************************************************************/
    /**
		 * @brief
		 */
    typedef MPU6050 * Ptr;

		/**
		 * @brief
		 */
		typedef MPU6050 * const CPtr;

		/** Constructors/destructors/overloads ************************************/
		/**
		 * @brief Constructor for the MPU6050-class.
		 *
		 * It also implements the I2C-library
		 * and sets up the I2C_ for communication.
		 *
		 * @param The I2C device address.
		 */
		explicit MPU6050(const uint8_t);

		/**
		 * @brief Copy-constructor for a MPU6050 instance.
		 * @param
		 *
		 * _deprecated_
		 */
		//MPU6050(const MPU6050&);

		/**
		 * @brief Destructor for a MPU6050 instance.
		 *
		 * _deprecated_
		 */
		//virtual ~MPU6050(void);

		/**
		 * @brief Operator overloading.
		 *
		 * _deprecated_
		 */
		//MPU6050& operator=(const MPU6050&);

		/** Settings **************************************************************/
		/**
		 * @brief Method for activating the MPU6050.
		 *
		 * @param gyroscopeScale The gyroscope scale setting (DEFAULT=0x00).
		 * @param acceleroScale The accelerometer scale setting (DEFAULT=0x00).
		 */
		void initialize(const uint8_t=0x00, const uint8_t=0x00);

		/**
		 * @brief Method for getting the scale-setting of the gyroscope.
		 *
		 * All possibilities are: 0x00 for +-250dps,
		 *    			  						0x08 for +-500dps,
		 *		 	  								0x10 for +-1000dps and
		 *			 								  0x18 for +-2000dps.
		 *
		 * @return gyroscopeScale_ The scale-setting of the gyroscope [byte].
		 */
		int8_t setGyroscopeScale(const uint8_t);

		/**
		 * @brief Method for getting the scale-setting of the accelero.
		 *
		 * All possibilities are: 0x00 for +-2g,
		 *    			  						0x08 for +-4g,
		 *			  								0x10 for +-8g and
		 *			  								0x18 for +-16g.
		 *
		 * @return acceleroScale_ The scale-setting of the accelerometer [byte].
		 */
		int8_t setAcceleroScale(const uint8_t);

		/** Runtime functions *****************************************************/
		/**
		 * @brief
		 */
		void updateGyroscopeBias(void) __attribute__ ((deprecated));

		/**
		 * Method for obtaining the gyroscope bias.
		 *
		 * @param[out] The bias vector (b_x, b_y, b_z)' in [rad/s].
		 */
		void updateGyroscopeBias(Vector&);

		/**
		 * Method for reading the measurement data from the gyroscope.
		 *
		 * NOTE::deprecated.
		 *
		 * @param[out] The omega vector (w_x, w_y, w_z)' pointed to in [rad/s].
		 */
		void updateGyroscope(Vector&) __attribute__ ((deprecated));

		/**
		 * @brief Method for reading the measurement data from the gyroscope.
		 *
		 * @param The bias vector (b_x, b_y, b_z)' in [rad/s].
		 * @param[out] The omega vector (w_x, w_y, w_z)' pointed to in [rad/s].
		 */
		void updateGyroscope(const Vector&, Vector&);

		/**
		 * @brief Method for reading the measurement data from the accelerometer.
		 *
		 * @param The raw acceleration vector (a_x, a_y, a_z)' pointed to in [g].
		 */
		void updateAccelero(Vector&);

		/**
		 * @brief Method for getting the gyroscope bias vector.
		 *
		 * NOTE::deprecated.
		 *
		 * @return _b The gyroscope bias vector [rad/s].
		 */
    Vector getGyroscopeBias(void) __attribute__ ((deprecated));

		/**
		 * @brief Method for getting the address of the MPU6050.
		 *
		 * @return The address of the I2C-device.
		 */
		uint8_t getAddress(void);

		/**
		 * @brief
		 */
		uint8_t getGyroscopeScale(void);

		/**
		 * @brief
		 */
		uint8_t getAcceleroScale(void);

  private:

		/** Variables *************************************************************/
		/**
		 * @brief Arduino TwoWire instance to communicate over I2C.
		 */
		TwoWire I2C_;

		/**
		 * @brief
		 */
		uint8_t address_;

		/**
		 * @brief
		 */
		uint8_t gyroscopeScale_;

		/**
		 * @brief
		 */
		uint8_t acceleroScale_;

		/**
		 * @brief The gyroscope bias vector. This bias depends on sensor drift,
		 *				environmental influences and so on. Hence, should be updated when
		 *				the copter has no rotational speed.
		 *
		 * NOTE::deprecated.
		 */
		Vector b_  __attribute__ ((deprecated));

		/** Statics ***************************************************************/
		/**
		 * @brief TODO::should be determined with a calibration method.
		 */
		double gyroConversionRate_ = 3754.94;

		/**
		 * @brief TODO::should be determined with a calibration method.
		 */
		double accConversionRate_ = 16384.0;

		/** Register locations ****************************************************/
		/**
		 * @brief TODO::Use evil C-preprocessor to change statics for different MPUs
		 */
		const static uint8_t pwrmgmt_ = 0x6B;

		/**
		 * @brief TODO::Use evil C-preprocessor to change statics for different MPUs
		 */
		const static uint8_t gyrcnfg_ = 0x1B;

		/**
		 * @brief TODO::Use evil C-preprocessor to change statics for different MPUs
		 */
		const static uint8_t acccnfg_ = 0x1C;

		/**
		 * @brief TODO::Use evil C-preprocessor to change statics for different MPUs
		 */
		const static uint8_t gyrdata_ = 0x43;
		/**
		 * @brief TODO::Use evil C-preprocessor to change statics for different MPUs
		 */
		const static uint8_t accdata_ = 0x3B;

}; //End MPU6050 class.

}; //End namespace component.

}; //End namespace qc.

#endif
