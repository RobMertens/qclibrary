#ifndef _PID_H_
#define _PID_H_

//#include <memory>
#include <stdint.h>

namespace qc
{

namespace component
{

/**
 * @brief
 */
enum class direction : int8_t
{
	FORWARD = 1,
	NONE		= 0,
	REVERSE = -1
};

/**
 * @brief
 */
class PID
{
	public:
		/**
		 * @brief
		 */
		typedef PID * Ptr;//std::shared_ptr<PID> ptr;

		/**
		 * @brief
		 */
		typedef PID * const Cptr;//std::shared_ptr<PID const> cptr;

		/**
		 * @brief
		 */
		PID(void);

		/**
		 * @brief
		 */
		PID(const double, const double, const double, const double, const double,
			const direction=direction::FORWARD);

		/**
		 * @brief
		 */
		void setOutputLimits(const double, const double);

		/**
		 * @brief
		 */
		void setGainValues(const double, const double, const double);

		/**
		 * @brief
		 */
		void setDirection(const direction);

		/**
		 * @brief
		 */
		void reset(void);

		/**
		 * @brief
		 */
		double calculate(const double, const double);

		/**
		 * @brief
		 */
		double getMinOutputLimit(void);

		/**
		 * @brief
		 */
		double getMaxOutputLimit(void);

		/**
		 * @brief
		 */
		double getProportionalGain(void);

		/**
		 * @brief
		 */
		double getIntegralGain(void);

		/**
		 * @brief
		 */
		double getDifferentialGain(void);

		/**
		 * @brief
		 */
		direction getDirection(void);

	private:
		/**
		 * @brief
		 */
		double _kp;                  							// (P)roportional Tuning Parameter

		/**
		 * @brief
		 */
		double _ki;                  							// (I)ntegral Tuning Parameter

		/**
		 * @brief
		 */
		double _kd;                  							// (D)erivative Tuning Parameter

		/**
		 * @brief
		 */
		double _input;

		/**
		 * @brief
		 */
		double _output;

		/**
		 * @brief
		 */
		double _desired;

		/**
		 * @brief
		 */
		double _maxLimit;

		/**
		 * @brief
		 */
		double _minLimit;

		/**
		 * @brief
		 */
		double _iterm;

		/**
		 * @brief
		 */
		double _lastError;

		/**
		 * @brief
		 */
		direction _direction;
};

}; //End namespace component.

}; //End namespace qc.

#endif
