#ifndef _PID_H_
#define _PID_H_

class PID
{
	public:
		//Constructors ***************************************************************
		PID(void);
		PID(float, float, float, float, float, uint8_t=FORWARD);

		//Setters ********************************************************************
		void setOutputLimits(float, float);
		void setGainValues(float, float, float);
		void setDirection(uint8_t);
							  
		//Getters ********************************************************************
		float calculate(float, float);
		float getMinOutputLimit();
		float getMaxOutputLimit();
		float getProportionalGain();
		float getIntegralGain();
		float getDifferentialGain();
		uint8_t getDirection();
		
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
		
		uint8_t _direction;
};
#endif

