/**
 *
 */

#include "logger.hpp"

namespace qc
{

/** Constructors/destructors/overloading **************************************/
Logger::Logger(const size_t baud, const SerialPacketConfig config) :
	baud_(baud), config_(config), logging_(false)
{
	#if defined(UBRRH) && defined(UBRRL)
  	serial_ = new HardwareSerial(&UBRRH, &UBRRL, &UCSRA, &UCSRB, &UCSRC, &UDR);
	#else
  	serial_ = new HardwareSerial(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0);
	#endif

}

/*Logger::~Logger(const Notification& note)
{
	delete serial_;
}*/

/** Serial monitor interfacing ************************************************/
void Logger::start(void)
{
	serial_->begin(baud_, config_);
	logging_ = true;
}

void Logger::stop(void)
{
	serial_->end();
	logging_ = false;
}

int8_t Logger::setBaudRate(const size_t baud)
{
	if(logging_)return -1;

	//Change the rate.
	baud_ = baud;

	return 0;
}

int8_t Logger::setSerialPacketConfig(const SerialPacketConfig config)
{
	if(logging_)return -1;

	//Change the rate.
	config_ = config;

	return 0;
}

/** Serial monitor interfacing ************************************************/
int8_t Logger::log(const Notification note, const char& m_string)
{
	//Check if we are logging.
	if(!logging_)return -1;

	//Create the header.
	time_t time_now = time(NULL);
	tm date_time = localtime( &time_now );
	Header header = { note, date_time, UNSPECIFIED } ;
	char* h_string = toString(header);

	//Print the stuff.
	serial_->println(h_string << m_string);

	//Success.
	return 0;
}

int8_t Logger::log(const Notification note, const math_helpers::Vector&, const char& m_string)
{

}

int8_t Logger::log(const Notification note, const math_helpers::Quaternion&, const char& m_string)
{

}

/** Helper functions **********************************************************/
char* Logger::toString(const Header& header)
{
	char* h_string;

	h_string = toString(header.note) << toString(header.date_time) <<
		toString(header.component);

	return h_string;
}

char* Logger::toString(const Notification& note)
{
	char* n_string;

	switch(note)
	{
		/* INFO */
		case Info :
			n_string = "[Info]";
			break;
		/* WARNING */
		case Warning :
			n_string = "[Warning]";
			break;
		/* ERROR */
		case Error :
			n_string = "[Error]";
			break;
	}

	return n_string;
}

char* Logger::toString(const Component& component)
{
	char* c_string;

	switch(component)
	{
		case UNSPECIFIED :
		case BAT :
		case ESC :
		case LED :
		case IMU :
		case PID :
		case RX :
		case NONE :
		default :

	}

	return c_string;
}

/**
 * @brief Create the global logger instance.
 */
Logger::CPtr logger = new Logger(0x2580);

}; //End namespace qc.
