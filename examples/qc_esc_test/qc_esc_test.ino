/******************************************************************************
 * Quadcopter-Library-v1
 * qc_esc_test.ino
 * 
 * This is a test file for the electronic speed controllers.
 *
 * @author:	Rob Mertens
 * @date:	07/05/2017
 * @version:	1.1.1
 ******************************************************************************/

#include "timer8.h"
#include "timer16.h"
#include "ESC.h"

/******************************************************************************
 * DECLARATIONS
 ******************************************************************************/
timer8 timer(t_alias::T2);
ESC esc(t_alias::T1, t_channel::B);

/******************************************************************************
 * MAIN
 ******************************************************************************/
int main(void)
{
	//Initialize.
	timer.initialize(t_mode::NORMAL, t_interrupt::OVF);
	timer.setPrescaler(1);
	esc.arm();
	
	//Run motors.
	esc.writeSpeed(0.5f);
	while(timer.getOverflows() >= 1000){;;}
	esc.writeMaxSpeed();
	while(timer.getOverflows() >= 1000){;;}
	esc.writeMinSpeed();
}
