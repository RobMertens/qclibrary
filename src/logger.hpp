



#ifndef qc_LOGGER_HPP
#define qc_LOGGER_HPP

//Include standard stuff.
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

//Include arduino stuff.
#include <HardwareSerial.h>

//Include qclibrary stuff.
#include "math_helpers.hpp"
//Include third-party library stuff.


namespace qc
{

/**
 * @brief For now, this class is just an overlay for the hardware serial.
 *
 * However, it actually should provide functions to store data during flight
 * and make them accessible afterwards.
 *
 *
 */
class Logger
{

	public:

		/**
		 *
		 */
		typedef Logger * Ptr;

		/**
		 *
		 */
		typedef Logger * const CPtr;

		/**
		 * @brief This constructor creates a logger instance with default a default
		 * 				baud rate of 9600 (0x2580) bits per second [bps].
		 * @param The desired baud rate [bps], DEFAULT : 9600.
		 */
		explicit Logger(const size_t=0x2580, const SerialPacketConfig=8N1);

		/**
		 *
		 */
		virtual ~Logger(void) { delete serial_; }

		/**
		 * @brief An enum containing all supported message types.
		 *
		 * IDEA::support specific message types for the specific components.
		 * 			 For example also to calibrate the RX-component.
		 *
		 * IDEA::support tables for the three default message types:
		 * 			 { Info, Warning, Error }
		 *			 By doing so, people can view the logs which appeared.
		 */
		enum Notification { Info=0, Warning=1, Error=2 };

		/**
		 *
		 */
		enum Component
		{
			NONE=-1,
			UNSPECIFIED=0,
			BAT=1,
			ESC=2,
			LED=3,
			IMU=4,
			PID=5,
			RX=6
		};

		/**
		 * @brief Descbribes the data structure of how data is sent to the serial_
		 * 				monitor. A typical data package contains three bit-types:
		 *				(1) Start bits 					{ 5, 6, 7, 8 },
		 *				(2) Parity (check) bits { N (None), E (Even), O (Odd) },
		 *				(3) Stop bits 					{ 1, 2 }.
		 */
		enum SerialPacketConfig
		{
			5N1=0x00, 6N1=0x02, 7N1=0x04,	8N1=0x06,
			5N2=0x08, 6N2=0x0A, 7N2=0x0C, 8N2=0x0E,
			5E1=0x20, 6E1=0x22, 7E1=0x24, 8E1=0x26,
			5E2=0x28, 6E2=0x2A, 7E2=0x2C, 8E2=0x2E,
			5O1=0x30, 6O1=0x32, 7O1=0x34, 8O1=0x36,
			5O2=0x38, 6O2=0x3A, 7O2=0x3C, 8O2=0x3E
		};

		/**
		 * @brief Method for starting the serial monitor. This function maps
		 *				the well known Serial.begin().
		 */
		void start(void);

		/**
		 * @brief Method for stopping the serial monitor.
		 */
		void stop(void);

		/**
		 * @brief
		 * @param
		 * @return
		 */
		int8_t setBaudRate(const size_t);

		/**
		 * @brief
		 * @param
		 * @return
		 */
		int8_t setSerialPacketConfig(const SerialPacketConfig);

		/**
		 * @brief
		 * @param
		 * @param
		 */
		void log(const Notification, const char&);

		/**
		 * @brief
		 * @param
		 * @param
		 * @param
		 */
		void log(const Notification, const math_helpers::Vector&, const char& m_string=null);

		/**
		 * @brief
		 * @param
		 * @param
		 * @param
		 */
		void log(const Notification, const math_helpers::Quaternion&, const char& m_string=null);

	private:

		/**
		 * @brief
		 */
		size_t baud_;

		/**
		 * @brief
		 */
		SerialPacketConfig config_;

		/**
		 * @brief
		 */
		bool logging_;

		/**
		 * @brief
		 */
		static HardwareSerial * serial_;

		/**
		 * @brief Struct containing the necessary ingredients for a logger header.
		 *				A header contains the following information in order:
		 *
		 * (1) A sequence number representing the unique message id.
		 *
		 * (2) A notification telling the message type. The current message types
		 * are {Info, Warning, Error}.
		 *
		 * (3) A component block telling from which component the message inherits.
		 */
		struct Header
		{
			size_t seq;
			Notification note;
			tm stamp;
			Component component;
		};

		/**
		 * @brief
		 */
		struct Footer
		{
			
		};

};

/**
 * @brief Create one logger intance for the whole project.
 */
extern Logger::CPtr logger;

}; //End namespace qc.

#endif //Endif wrapper qc_LOGGER_HPP.
