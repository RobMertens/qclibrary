#ifndef _BAT_H_
#define _BAT_H_

class BAT
{
	public:
		//Constructors ***************************************************************
		BAT(void);

		//Setters ********************************************************************

		//Getters ********************************************************************
		uint8_t getBatteryLevel(void);

	private:
		uint8_t _level;								// Variable with LOW LED-state.

};
#endif

