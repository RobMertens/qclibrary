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
#include "definitions.hpp" //MODIFY THIS FILE!
#include "hardware/MPU6050.hpp"
#include "hardware/RX.hpp"
#include "hardware/PID.hpp"
#include "hardware/ESC.hpp"
#include "controller.hpp"
#include "hardware/LED.hpp"

/******************************************************************************
 * DECLARATIONS
 ******************************************************************************/
//Hardware components.
MPU6050::Ptr imu(new MPU6050(MPU_ADDRESS)); //TODO::(MPU_DPS_SCALE, MPU_ACC_SCALE)
RX::Ptr receiver(new RX(&RX_PCMSK, RX_PCINT, alias::T2, RX_PERIOD, RX_MAX, RX_MIN));
PID::Ptr roll(new PID(P_R_KP, P_R_KI, P_R_KD, P_R_MAX, P_R_MIN));
PID::Ptr pitch(new PID(P_P_KP, P_P_KI, P_P_KD, P_P_MAX, P_P_MIN));
PID::Ptr yaw(new PID(P_Y_KP, P_Y_KI, P_Y_KD, P_Y_MAX, P_Y_MIN));
ESC::Ptr drive1(new ESC(t_settings::alias::T1, t_settings::channel::B, E_PERIOD, E_MAX, E_MIN));
ESC::Ptr drive2(new ESC(t_settings::alias::T3, t_settings::channel::C, E_PERIOD, E_MAX, E_MIN));
ESC::Ptr drive3(new ESC(t_settings::alias::T1, t_settings::channel::B, E_PERIOD, E_MAX, E_MIN));
ESC::Ptr drive4(new ESC(t_settings::alias::T3, t_settings::channel::C, E_PERIOD, E_MAX, E_MIN));

//Flight controller.
//controller::Ptr con(new controller(c_settings::layout::CROSS, t_settings::alias::T2));

//LED indicator.
LED::Ptr led(new LED(L_DDR, L_DDRMSK, L_PIN, L_PORT));

/******************************************************************************
 * MAIN LOOP.
 ******************************************************************************/
int main(void)
{
	//Assign hardware.
	led->set();
	//con->assignImu(imu);
	//con->assignReceiver(receiver, rx_settings::mode::M1);
	//con->assignPids(roll, pitch, yaw);
	//con->assignDrives(drive1, drive2, drive3, drive4);

	//Flight controller init.
	//con->initialize();
	led->reset();

	for (;;)
	{
		//Flight controller update.
		//con->update();

		//Battery monitor
		//if(!con->monitorBattery())led->set();
	}
}
