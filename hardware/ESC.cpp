/******************************************************************************
 * Quadcopter-Library-v1
 * ESC.cpp
 *  
 * This file contains predefined functions for the ESC-class. These functions
 * serve for driving the motors of the quadcopter. They do this by writing
 * PWM-signals to the electronic speed controllers (ESC's). Every PWM-signal
 * takes about 4000 microseconds (HIGH and LOW part) which directly gives us 
 * the refreshrate of the controller:
 * 
 * refreshrate = 1/0.004 = 250 Hz.
 *
 * PLEASE NOTE: This class is based on arduino's which use a clock frequency 
 *		of 16Mhz. Other arduino's will NOT generate a 4000us signal.
 * 
 * TODO::add number of motors.
 * TODO::variable frequency and clock speed.
 * 
 * In this class predefined registers are used. For each hardware-setup
 * these registers can be different. It is recommended to change the registers
 * in the DEFINE.h file.
 *
 * @author: 	Rob Mertens
 * @date:	14/08/2016
 * @version: 	1.0.1
 ******************************************************************************/

#include <avr/io.h>
#include <ESC.h>

/*******************************************************************************
 * Constructor for the ESC-class.
 * 
 * By making an object with this constructor all local variables are set together
 * with the avr-timer (DEFAULT=TIMER1). This constructor is compatible for most
 * ESC's which have a DEFAULT microsecond PMW-range of minimum 1000us (motors off)
 * and maximum 2000us (motors at maximum throttle) or user specified.
 * 
 * @param: *ddr The Data Direction Register.
 * @param: mask The Data Direction Register mask.
 * @param: *pin The Port Input register.
 * @param: *port The Port Output Register.
 * @param: *tccr The Timer Count Control Register (DEFAULT=&TCCR1).
 * @param: *tcnt The Timer Count Register (DEFAULT=&TCNT1).
 * @param: *timsk The Timer Mask Register (DEFAULT=&TIMSK1).
 * @param: minMicrosecond The zero throttle pulse length [µs] (DEFAULT=1000).
 * @param: maxMicrosecond The full throttle pulse length [µs] (DEFAULT=2000).
 ******************************************************************************/
ESC::ESC(volatile uint8_t * ddr, uint8_t mask, volatile uint8_t * pin, volatile uint8_t * port, volatile uint8_t * tccr=&TCCR1, volatile uint16_t * tcnt=&TCNT1, volatile uint8_t * timsk=&TIMSK1, int minMicrosecond=1000, int maxMicrosecond=2000)
{	
	// IO.
	_ddr  = &ddr;							// Put definitions in local variables.
	_pin  = &pin;
	_port = &port;

	*_ddr |= mask;
	
	// ESC Timer.
	_escTimer = timer(&tccr, &tcnt, &timsk)
	
	//
	_HIGH = *_ddr;
	_LOW  = *_ddr ^ 0xFF;

	_minTicks = microseconds2Ticks(minMicrosecond);
	_maxTicks = microseconds2Ticks(maxMicrosecond);

	for (uint8_t mask=0x01; mask<=0x80; mask<<=1)			// Loop through every bit in a byte.
	{
		int n = 0;
		if(*_ddr & mask)
		{
			n++;
			if(n==1)_esc1 |= mask;
			if(n==2)_esc2 |= mask;
			if(n==3)_esc3 |= mask;
			if(n==4)_esc4 |= mask;
		} 
	}
}

/*******************************************************************************
 * Method for initializing the timer-settings.
 * The default is TIMER1 with no prescaler and timer overflow interrupt.
 *
 * @param: prescale The prescale mask value (DEFAULT=0x01).
 * @param: mask The timsk mask value (DEFAULT=0x01).
 ******************************************************************************/
void ESC::arm(uint8_t prescale=0x01, uint8_t mask=0x01)
{
	// TODO::Asking a prescale and timsk here is odd.
	//	 Autocomputation timer in function of timer length (8/16-bit).
	_escTimer.initialize(prescale, mask)				// DEFAULT TIMER1:
									// Set up the 16-bit timer prescaler value 0.
    									// Thus our timer resolution equals:
    									// (absolute) = 1/16000000 = 0.0825µs
    									// (relative) = 0.0825/4000 = negligible...
	
	_us2t = (1/1000000)/(prescale/16000000);			// ticks = (1 / desired frequency) / (prescaler / clock speed) - 1.
									// TODO::variable frequency and clock speed.
}

/*******************************************************************************
 * Method for writing variable PWM-pulses in length to the ESC's.
 *
 * REMARK: This method takes 4000µs in time.
 * TODO::use interrupts to do other things.
 *	
 * @param: us1 The pulselength for ESC1 in microseconds.
 * @param: us2 The pulselength for ESC2 in microseconds.
 * @param: us3 The pulselength for ESC3 in microseconds.
 * @param: us4 The pulselength for ESC4 in microseconds.
 ******************************************************************************/
void ESC::writeVariableSpeed(int us1, int us2, int us3, int us4)
{
	*_port |= _HIGH;						// Set ouput HIGH.
	_escTimer.reset();						// Reset timer.
	
	int tck1 = microseconds2Ticks(us1);				// Transform microseconds to timer compatible ticks.
	int tck2 = microseconds2Ticks(us2);
	int tck3 = microseconds2Ticks(us3);
	int tck4 = microseconds2Ticks(us4);
	
	while((_escTimer.getNonResetCount()) < _maxTicks)
	{		
		if((_escTimer.getNonResetCount()) >= tck1 && *_pin & _esc1){*_port &= (_esc1 ^ 0xFF);}
		if((_escTimer.getNonResetCount()) >= tck2 && *_pin & _esc2){*_port &= (_esc2 ^ 0xFF);}
		if((_escTimer.getNonResetCount()) >= tck3 && *_pin & _esc3){*_port &= (_esc3 ^ 0xFF);}
		if((_escTimer.getNonResetCount()) >= tck4 && *_pin & _esc4){*_port &= (_esc4 ^ 0xFF);}
	}
}

/*******************************************************************************
 * Method for writing LOW PWM-pulses to the ESC's.
 ******************************************************************************/
void ESC::writeMinimumSpeed()
{
	*_port |= _HIGH;						// Set ouput HIGH.
	TCNT0 = 0; 							// Reset timer.
	uint16_t timerOverflow = 0x0000;				// Private overflow word.
	
	int tck = _minTicks;						// High pulse takes minimum microseconds.
	
	while((timerOverflow + TCNT0) < _maxTicks)			// Total time takes 4000 microseconds.
	{		
		if(TIFR0 & 0x01)					// Overflow occured
		{
			timerOverflow += 0xFF;				// Add maximum timervalue to overflow word.
			TIFR0 |= 0x01;					// Reset overflow bit.
		}
		if((timerOverflow + TCNT0) >= tck && *_pin & _HIGH)*_port &= _LOW;
	}
}

/*******************************************************************************
 * Method for writing HIGH PWM-pulses to the ESC's.
 ******************************************************************************/
void ESC::writeMaximumSpeed()
{
	*_port |= _HIGH;						// Set ouput HIGH.
	TCNT0 = 0; 							// Reset timer
	uint16_t timerOverflow = 0x0000;				// Private overflow word.
	
	int tck = _maxTicks;						// High pulse takes maximum microseconds.
	
	while((timerOverflow + TCNT0) < _maxTicks)			// Total time takes 4000 microseconds.
	{		
		if(TIFR0 & 0x01)					// Overflow occured
		{
			timerOverflow += 0xFF;				// Add maximum timervalue to overflow word.
			TIFR0 |= 0x01;					// Reset overflow bit.
		}
		if((timerOverflow + TCNT0) >= tck && *_pin & _HIGH){*_port &= _LOW;}
	}
}

/*******************************************************************************
 * Private method for transforming the a microsecond value to a timer
 * compatible value. It also contains a safety check on minimum and maximum
 * values.
 *
 * @param: us The length of the PWM-pulse in microseconds.
 * @return: tck The lenght of the PWM-pulse in timer compatible ticks.
 ******************************************************************************/
int ESC::microseconds2Ticks(int us)
{
	int tck;
	
	tck = us*_us2t - 1;							// ticks = (1 / desired frequency) / (prescaler / clock speed) - 1.
	if(tck > _maxTicks){tck = _maxTicks;}
	else if(tck < _minTicks){tck = _minTicks;}
	
	return tck;
}
