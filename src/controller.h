#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdint.h>
#include "math.h"
#include "settings.h"
#include "timer8.h"
#include "timer16.h"
#include "vmath.h"
#include "hardware/MPU6050.h"
#include "hardware/PID.h"
#include "hardware/RX.h"
#include "hardware/ESC.h"
#include "hardware/LED.h"

namespace controller
{
	/*****************************************************************************
	 * @brief Layout.
	 ****************************************************************************/
	enum class layout : uint8_t
	{
		NONE	= 0,
		PLUS	= 1,
		CROSS	= 2
	};

	/*****************************************************************************
	 * @brief Mode.
	 ****************************************************************************/
	enum class mode : uint8_t
	{
		NONE	= 0,
		ATT	= 1,
		VEL	= 2,	// GPS Needed.
		POS	= 3,	// GPS Needed.
		HVR	= 4
	};

	/*******************************************************************
	 * @brief Drone flying safety state.
 	 * 				TODO::more error states -> specific errors.
	 ******************************************************************/
	enum class safety : int8_t
	{
		E1	= -1,						// Error state.
		OFF = 0,						// Not flying.
		T1	= 1,						// Transition state OFF -> ON.
		ON	= 2,						// Flying.
		T2	= 3,						// Transition state ON -> OFF.
	};

	/*****************************************************************************
	 * @brief Inputs.
	 ****************************************************************************/
	struct inputs
	{
		//Constructors *************************************************************
		inputs(void) {reset();}

		//Setters ******************************************************************
		void reset(void) {throttle = 0.0f;roll = 0.0f;pitch = 0.0f;yaw = 0.0f;}

		//Variables ****************************************************************
		float throttle;
		float roll;
		float pitch;
		float yaw;
		//float extra;
	};

	/*****************************************************************************
	 * @brief Outputs.
	 ****************************************************************************/
	struct outputs
	{
		//Constructors *************************************************************
		outputs(void) {reset();}

		//Setters ******************************************************************
		void reset(void) {esc1 = 0.0f;esc2 = 0.0f;esc3 = 0.0f;esc4 = 0.0f;}

		//Variables ****************************************************************
		float esc1;
		float esc2;
		float esc3;
		float esc4;
	};

	/*****************************************************************************
	 * @brief State vector.
	 ****************************************************************************/
	struct state
	{
		//Constructors *************************************************************
		state(void)
		: localOmega(new vector()),
			localAccel(new vector()),
			worldAccel(new vector()),
			local_R_world(new quaternion()) {}

		//Setters ******************************************************************
		void reset(void)
		{
			*localOmega 	 = vector();
			*localAccel		 = vector();
			*worldAccel 	 = vector();
			*local_R_world = quaternion();
		}

		//Variables ****************************************************************
		vector::ptr localOmega;
		vector::ptr localAccel;
		vector::ptr worldAccel;
		quaternion::ptr local_R_world;
	};

	//Setters ********************************************************************
	void updateReceiverInputs(const RX::cptr&, inputs&);
	void updateDesiredState(const inputs&, state&);
	void updateSafetyState(const inputs&, safety&);
	void updateLocalData(const MPU6050::cptr&, state&);
	void updateActualAttitude(const float, state&);
	void updateWorldData(state&);
	void updateOutputs(const PID::cptr&, const PID::cptr&, const PID::cptr&,
		const inputs&, const state&, const state&, const layout&, outputs&);
	void resetOutputs(PID::cptr& roll, PID::cptr& pitch, PID::cptr& yaw,
		outputs& outputs);
	void driveMotors(ESC::cptr& esc1, ESC::cptr& esc2, ESC::cptr& esc3,
		ESC::cptr& esc4, const outputs& outputs);
	void stopMotors(void);
	void enableMotors(void);
	void disableMotors(void);

	//Getters ********************************************************************
	bool getMovement(const vector::cptr& accel, const float p, float& eta=0.0f);

}; //End namespace constoller;

#endif
