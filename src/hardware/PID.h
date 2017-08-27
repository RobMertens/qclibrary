#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>

class PID
{
	public:
		//Constructors *************************************************************
		PID(void);
		PID(const float, const float, const float, const float, const float, const int8_t=0x01);

		//Setters ******************************************************************
		void setOutputLimits(const float, const float);
		void setGainValues(const float, const float, const float);
		void setDirection(const int8_t);
		void reset(void);

		//Getters ******************************************************************
		float calculate(const float, const float);
		float getMinOutputLimit(void);
		float getMaxOutputLimit(void);
		float getProportionalGain(void);
		float getIntegralGain(void);
		float getDifferentialGain(void);
		int8_t getDirection(void);

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

		int8_t _direction;
};

#endif
