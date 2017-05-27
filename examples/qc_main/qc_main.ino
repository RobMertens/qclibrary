/******************************************************************************
 * Quadcopter-Library-v1
 * qc_main.ino
 *
 * @author:	Rob Mertens
 * @date:	10/05/2017
 * @version:	1.1.1
 ******************************************************************************/
#include "controller.h"
#include "definitions.h" //MODIFY THIS FILE!

/******************************************************************************
 * DECLARATIONS
 ******************************************************************************/
RX receiver(&RX_PCMSK, RX_PCINT, t_alias::T4, RX_PERIOD, RX_MAX, RX_MIN);

MPU imu(MPU_ADDRESS, t_alias::T5, MPU_DPS_SCALE, MPU_ACC_SCALE);

PID  roll(P_ROLL_KP,  P_ROLL_KI,  P_ROLL_KD,  P_ROLL_MAX,  P_ROLL_MIN);
PID pitch(P_PITCH_KP, P_PITCH_KI, P_PITCH_KD, P_PITCH_MAX, P_PITCH_MIN);
PID   yaw(P_YAW_KP,   P_YAW_KI,   P_YAW_KD,   P_YAW_MAX,   P_YAW_MIN);

ESC drive1(t_alias::T1, t_channel::B, E_PERIOD, E_MAX, E_MIN);
ESC drive2(t_alias::T3, t_channel::C, E_PERIOD, E_MAX, E_MIN);
ESC drive3(t_alias::T1, t_channel::B, E_PERIOD, E_MAX, E_MIN);
ESC drive4(t_alias::T3, t_channel::C, E_PERIOD, E_MAX, E_MIN);

controller con(imu, receiver, roll, pitch, yaw, drive1, drive2, drive3, drive4, c_layout::CROSS, t_alias::T2);

LED led(&L_DDR, L_DDRMSK, &L_PIN, &L_PORT);

/******************************************************************************
 * MAIN LOOP.
 ******************************************************************************/
int main(void)
{
	//Serial.
	Serial.begin(115200);
	
	//Flight controller init.
	led.set();
	con.initialize();
	led.reset();
	
	for (;;)
	{
		//Flight controller update.
		con.update();
		
		//Battery monitor
		if(!con.monitorBattery())led.set();
		
		//Looptime monitor
		Serial.println(con.getLooptime());
	}
}
