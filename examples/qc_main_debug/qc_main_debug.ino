/******************************************************************************
 * Quadcopter-Library-v1
 * qc_main.ino
 * 
 * TODO::add battery.
 * TODO::add all debug functions in controller as doc.
 * 
 * @author:	Rob Mertens
 * @date:	10/05/2017
 * @version:	1.1.1
 ******************************************************************************/
#include "definitions.h" //MODIFY THIS FILE!
#include "hardware/MPU6050.h"
#include "hardware/RX.h"
#include "hardware/PID.h"
#include "hardware/ESC.h"
#include "controller.h"
#include "hardware/LED.h"

/******************************************************************************
 * DECLARATIONS
 ******************************************************************************/
//Hardware components.
MPU imu(MPU_ADDRESS, MPU_DPS_SCALE, MPU_ACC_SCALE);

RX receiver(&RX_PCMSK, RX_PCINT, t_alias::T2, RX_PERIOD, RX_MAX, RX_MIN);

PID r(P_R_KP, P_R_KI, P_R_KD, P_R_MAX, P_R_MIN);
PID p(P_P_KP, P_P_KI, P_P_KD, P_P_MAX, P_P_MIN);
PID y(P_Y_KP, P_Y_KI, P_Y_KD, P_Y_MAX, P_Y_MIN);

ESC drive1(t_alias::T1, t_channel::B, E_PERIOD, E_MAX, E_MIN);
ESC drive2(t_alias::T3, t_channel::C, E_PERIOD, E_MAX, E_MIN);
ESC drive3(t_alias::T1, t_channel::B, E_PERIOD, E_MAX, E_MIN);
ESC drive4(t_alias::T3, t_channel::C, E_PERIOD, E_MAX, E_MIN);

//Flight controller.
controller con(c_layout::CROSS, t_alias::T2);

//LED indicator.
LED led(&L_DDR, L_DDRMSK, &L_PIN, &L_PORT);

/******************************************************************************
 * MAIN LOOP.
 ******************************************************************************/
int main(void)
{
	//Assign hardware.
	led.set();
	con.assignImu(&imu);
	con.assignReceiver(&rec, rx_mode::M1);
	con.assignPids(&r, &p, &y);
	con.assignDrives(&esc1, &esc2, &esc3, &esc4);
	
	//Flight controller init.
	con.initialize();
	led.reset();
	
	for (;;)
	{
		//Flight controller update.
		con.update();
		
		//Battery monitor
		if(!con.monitorBattery())led.set();
	}
}
