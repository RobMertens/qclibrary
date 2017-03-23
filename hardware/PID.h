#ifndef _PID_H_
#define _PID_H_

class PID
{

  public:

  //Constructors ***************************************************************
    PID(void);
  	PID(double, double, double, int, double, double);     //   Setpoint.  Initial tuning parameters are also set here

  //Setters ********************************************************************
    void setOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application
	
	void setGainValues(double, double, double);
	
	void setDirection(int);
						  
  //Getters ********************************************************************
	double calculate(double, double);
    double getMinOutputLimit();
	double getMaxOutputLimit();
  	double getProportionalGain();						  // These functions query the pid for interal values.
	double getIntegralGain();						  //  they were created mainly for the pid front-end,
	double getDifferentialGain();						  // where it's important to know what is actually 
	int getDirection();

  private:  

	int _direction;

	double _kp;                  // * (P)roportional Tuning Parameter
    double _ki;                  // * (I)ntegral Tuning Parameter
    double _kd;                  // * (D)erivative Tuning Parameter
    
    double _input;
    double _output;
    double _desired;
	double _iterm, _lastError;

	double _minLimit, _maxLimit;
};
#endif

