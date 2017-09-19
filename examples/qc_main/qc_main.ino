/******************************************************************************
 * Quadcopter-Library-v1
 * qc_main.ino
 *
 * TODO::add battery.
 * TODO::add all debug functions in controller as doc.
 *
 * @author:		Rob Mertens
 * @date:			10/05/2017
 * @version:	1.1.1
 ******************************************************************************/
#include "definitions.h" //MODIFY THIS FILE!
#include "hardware/MPU6050.h"
#include "hardware/RX.h"
#include "hardware/PID.h"
#include "hardware/ESC.h"
#include "hardware/LED.h"
#include "controller.h"

using namespace controller;

/******************************************************************************
 * DECLARATIONS
 ******************************************************************************/
//Hardware components.
MPU6050::ptr imu(new MPU6050(MPU_ADDRESS));
RX::ptr receiver(new RX(&RX_PCMSK, RX_PCINT, RX_T_ALIAS, RX_PERIOD, RX_MAX, RX_MIN));
PID::ptr roll(new PID(P_R_KP, P_R_KI, P_R_KD, P_R_MAX, P_R_MIN));
PID::ptr pitch(new PID(P_P_KP, P_P_KI, P_P_KD, P_P_MAX, P_P_MIN));
PID::ptr yaw(new PID(P_Y_KP, P_Y_KI, P_Y_KD, P_Y_MAX, P_Y_MIN));
ESC::ptr drive1(new ESC(E1_T_ALIAS, E1_T_CHANNEL, E_PERIOD, E_MAX, E_MIN));
ESC::ptr drive2(new ESC(E2_T_ALIAS, E2_T_CHANNEL, E_PERIOD, E_MAX, E_MIN));
ESC::ptr drive3(new ESC(E3_T_ALIAS, E3_T_CHANNEL, E_PERIOD, E_MAX, E_MIN));
ESC::ptr drive4(new ESC(E4_T_ALIAS, E4_T_CHANNEL, E_PERIOD, E_MAX, E_MIN));
LED::ptr led(new LED(L_DDR, L_DDRMSK, L_PIN, L_PORT));

//Watchdog.
timer16 watchdog;				//TODO::timer16::ptr and timer16::cptr

//Controller states.
inputs inputs;
outputs outputs;
state actual, desired;
safety state;

/*******************************************************************************
 * MAIN.
 ******************************************************************************/
int main(void)
{
	/*****************************************************************************
	 * SETUP.
	 ****************************************************************************/
	//Start up indication.
	led->set();

	//Assign hardware.
	imu->initialize(MPU_DPS_SCALE, MPU_ACC_SCALE);
	receiver->initialize(RX_MODE);
	esc1->arm(E_T_PRESCALER, E_T_TOP);
	esc2->arm(E_T_PRESCALER, E_T_TOP);
	esc3->arm(E_T_PRESCALER, E_T_TOP);
	esc4->arm(E_T_PRESCALER, E_T_TOP);

	//Watchdog.
	uint32_t looptime = 0;
	watchdog = timer16(W_T_ALIAS);
	watchdog.initialize(W_T_MODE, W_T_INTERRUPT);
	watchdog.setPrescaler(W_T_PRESCALER);

	//Start up succes.
	led->reset();

	/*****************************************************************************
	 * LOOP.
	 * Update method of the controller. This method sequentially handles the next
	 * functions:
	 * (1) Update & process receiver inputs.
	 * (2) Monitor the quadcopters' safety state.
	 * (3) Update & process sensor inputs.
	 * (4) PID feedback control.
	 * (5) Drive the motors according to safety state.
	 * (6) Additional functions.
	 ****************************************************************************/
	for (;;)
	{
		//(1) Update receiver inputs.
		updateReceiverInputs(receiver, inputs);
		updateDesiredState(inputs, desired);

		//(2) Quadcopter safety state.
		updateSafetyState(inputs, state);

		//(3) Update measurements.
		updateActualState(imu, looptime, actual)

		//(4) PID feedback.
		if(state==safety::ON)
		{
			updateOutputs(roll, pitch, yaw, inputs, actual, desired, layout, outputs);
		}
		else
		{
			resetOutputs(roll, pitch, yaw, outputs);
		}

		//(5) Drive motors according to safety state.
		driveMotors(esc1, esc2, esc3, esc4, outputs);

		//(6) Additional functions.
		//TODO::time calculations in timer class.
		looptime = (uint32_t)(watchdog.getNonResetCount()*0.0625f*64.0f*US);
		watchdog.reset();

		//Battery monitor
		if(!con->monitorBattery())led->set();
	}
};
