#ifndef _BAT_H_
#define _BAT_H_

//#include <memory>
#include <stdint.h>

class BAT
{
	public:
		//Typedefs *****************************************************************
		typedef BAT * ptr; //std::shared_ptr<BAT> ptr;
		typedef BAT * const cptr; //std::shared_ptr<BAT const> cptr;

		//Constructors ***************************************************************
		BAT(void);

		//Setters ********************************************************************

		//Getters ********************************************************************
		uint8_t getLevel(void);

	private:
		uint8_t _level;								// Variable with LOW LED-state.

};
#endif
