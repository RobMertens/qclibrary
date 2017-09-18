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

namespace c_settings
{
	/*****************************************************************************
	 * @brief TODO::this is future stuff.
	 ****************************************************************************/
	enum class mode : uint8_t
	{
		NONE	= 0,
		ATT	= 1,
		VEL	= 2,	// GPS Needed.
		POS	= 3,	// GPS Needed.
		HVR	= 4
	};

	enum class layout : uint8_t
	{
		NONE	= 0,
		PLUS	= 1,
		CROSS	= 2
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

	/*******************************************************************
	 * @brief
 	 * @param
	 ******************************************************************/
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
	 * @brief
	 * @param
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
	 * @brief Drone state 40-entry vector. A lot of vectors are unused since
	 *				this requires more sensors.
 	 * 				TODO::does this belong in "c_settings"?
   * 			  [[   			 	(1);					(2); 				 	(3)]
	 * 				 [=============;=============;=============]
	 *				 [  theta_local;	omega_local;	alpha_local]
	 * 				 [   			 	(3);					(4); 				 	(5)]
	 * 				 [=============;=============;=============]
	 *				 [  theta_world;	omega_world;	alpha_world]
	 * 				 [   			 	(6);					(7); 				 	(8)]
	 * 				 [=============;=============;=============]
	 *				 [  	pos_local;	 	vel_local;	accel_local]
	 * 				 [   			 	(9);				 (10); 				 (11)]
	 * 				 [=============;=============;=============]
	 *				 [  	pos_world;	 	vel_world;	accel_world]
	 * 				 [   			 (12)]
	 * 				 [=============]
	 *				 [  	 attitude]];
	 ****************************************************************************/
	struct state
	{
		//Constructors *************************************************************
		state(void) : localOmega(new vector()), localAccel(new vector()),
			worldAccel(new vector()), local_R_world(new quaternion()) {}

		//Setters ******************************************************************
		void reset(void)
		{
			*localOmega 	 = vector();
			*localAccel		 = vector();
			*worldAccel 	 = vector();
			*local_R_world = quaternion();
		}

		//Variables ****************************************************************
		//vector::ptr localTheta;																										//Rotations.
		vector::ptr localOmega;
		//vector::ptr localAlpha;
		//vector::ptr worldTheta;
		//vector::ptr worldOmega;
		//vector::ptr worldAlpha;
		//vector::ptr localPos;																											//Translations.
		//vector::ptr localVel;
		vector::ptr localAccel;
		//vector::ptr worldPos;
		//vector::ptr worldVel;
		vector::ptr worldAccel;
		quaternion::ptr local_R_world;																							//Rotation matrix.
	};

}; //End namespace c_settings.

using namespace c_settings;

class controller
{
	public:
		//Typedefs *****************************************************************
		typedef controller * ptr;
		typedef controller * const cptr;

		//Constructors *************************************************************
		controller(const c_settings::layout&, const t_settings::alias&);

		//Setters ******************************************************************
		void assignImu(const MPU6050::cptr&);
		void assignReceiver(const RX::cptr&, const rx_settings::mode);
		void assignPids(const PID::cptr&, const PID::cptr&, const PID::cptr&);
		void assignDrives(const ESC::cptr&, const ESC::cptr&, const ESC::cptr&,
			const ESC::cptr&);
		void initialize(void);
		void update(void);
		void stopMotors(void);
		void enableMotors(void);
		void disableMotors(void);

		//Getters ******************************************************************
		bool getMovement(const vector::cptr& accel, const float p, float& eta);
		float getLooptime(void);
		int8_t monitorBattery(void);
		safety getSafetyState(void);

	protected:
		//Quadcopter attributes ****************************************************
		ESC::ptr _esc1;
		ESC::ptr _esc2;
		ESC::ptr _esc3;
		ESC::ptr _esc4;
		PID::ptr _pidRoll;
		PID::ptr _pidPitch;
		PID::ptr _pidYaw;
		RX::ptr _rec;
		MPU6050::ptr _imu;

	private:
		//Variables ****************************************************************
		float _looptime;
		layout _layout;
		timer16 _watchdog;		//Watchdog timer for checking calculations time.

		//Quadcopter state vector **************************************************
		inputs _inputs;
		safety _safety;
		state _actual;
		state _desired;
		outputs _outputs;

		//Setters ******************************************************************
		void updateReceiverData(const RX::cptr&, inputs&);
		void updateDesiredAttitude(const inputs&, quaternion::cptr&);
		void updateSafetyState(const inputs&, safety&);
		void updateLocalData(const MPU6050::cptr&, const safety&, vector::cptr&,
			vector::cptr&);
		void updateActualAttitude(const vector::cptr&, const vector::cptr&,
			const float, const quaternion::cptr&, quaternion::cptr& output);
		void updateWorldData(const vector::cptr&, const quaternion::cptr&,
			vector::cptr&);
		void updateOutputs(const PID::cptr&, const PID::cptr&, const PID::cptr&,
			const inputs&, const quaternion::cptr&, const quaternion::cptr&,
			const layout&, outputs&);
		void driveMotors(const outputs& outputs, ESC::cptr& esc1, ESC::cptr& esc2,
			ESC::cptr& esc3, ESC::cptr& esc4);
		void resetOutputs(PID::cptr& roll, PID::cptr& pitch, PID::cptr& yaw,
			outputs& outputs);
		void updateActualAttitude(const vector::cptr&, const vector::cptr&,
			const quaternion::cptr&, quaternion::cptr&);
		bool getMovement(const vector::cptr&, const float);

		//Deprecated ***************************************************************
		const static float _DEADBAND;
		const static float _DEG2RAD;
		const static float _RAD2DEG;
		const static vector::cptr _UNITX;
		const static vector::cptr _UNITY;
		const static vector::cptr _UNITZ;
		const static vector::cptr _MAXANGLE;
		const static quaternion::cptr _REAL;

};
#endif
