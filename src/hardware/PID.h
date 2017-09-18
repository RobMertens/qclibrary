#ifndef _PID_H_
#define _PID_H_

//#include <memory>
#include <stdint.h>

namespace pid_settings
{
	enum class direction : int8_t
	{
		FORWARD = 1,
		NONE		= 0,
		REVERSE = -1
	};
};

using namespace pid_settings;

class PID
{
	public:
		//Typedefs *****************************************************************
		typedef PID * ptr;//std::shared_ptr<PID> ptr;
		typedef PID * const cptr;//std::shared_ptr<PID const> cptr;

		//Constructors *************************************************************
		PID(void);
		PID(const float, const float, const float, const float, const float,
			const direction=direction::FORWARD);

		//Setters ******************************************************************
		void setOutputLimits(const float, const float);
		void setGainValues(const float, const float, const float);
		void setDirection(const direction);
		void reset(void);

		//Getters ******************************************************************
		float calculate(const float, const float);
		float getMinOutputLimit(void);
		float getMaxOutputLimit(void);
		float getProportionalGain(void);
		float getIntegralGain(void);
		float getDifferentialGain(void);
		direction getDirection(void);

	private:
		float _kp;                  							// (P)roportional Tuning Parameter
		float _ki;                  							// (I)ntegral Tuning Parameter
		float _kd;                  							// (D)erivative Tuning Parameter

		float _input;
		float _output;
		float _desired;
		float _maxLimit;
		float _minLimit;
		float _iterm;
		float _lastError;

		direction _direction;
};

#endif
