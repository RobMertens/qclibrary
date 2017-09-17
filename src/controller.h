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

	/*****************************************************************************
	 * @brief Drone flying safety state.
 	 * 				TODO::more error states -> specific errors.
	 ****************************************************************************/
	enum class safety : int8_t
	{
		E1	= -1,						// Error state.
		OFF = 0,						// Not flying.
		T1	= 1,						// Transition state OFF -> ON.
		ON	= 2,						// Flying.
		T2	= 3,						// Transition state ON -> OFF.
	};

	/*****************************************************************************
	 * @brief Drone state 40-entry vector. A lot of vectors are unused since
	 *				this requires more sensors.
 	 * 				TODO::does this belong in "c_settings"?
   * 			 [[   			 (1);					 (2); 				 (3)]
	 * 				[=============;=============;=============]
	 *				[  theta_local;	 omega_local;	 alpha_local]
	 * 				[   			 (3);					 (4); 				 (5)]
	 * 				[=============;=============;=============]
	 *				[  theta_world;	 omega_world;	 alpha_world]
	 * 				[   			 (6);					 (7); 				 (8)]
	 * 				[=============;=============;=============]
	 *				[  	 pos_local;	 	 vel_local;	 	 acc_local]
	 * 				[   			 (9);					(10); 				(11)]
	 * 				[=============;=============;=============]
	 *				[  	 pos_world;	 	 vel_world;	 	 acc_world]
	 * 				[   			(12);]
	 * 				[=============;]
	 *				[  	  attitude;]];
	 ****************************************************************************/
	struct state
	{
		state(void)
		{
			//Rotations.
			_localTheta = &(new vector());
			_localOmega = &(new vector());
			_localAlpha = &(new vector());
			_worldTheta = &(new vector());
			_worldOmega = &(new vector());
			_worldAlpha = &(new vector());
			//Translations.
			_localPos = &(new vector());
			_localVel = &(new vector());
			_localAcc = &(new vector());
			_worldPos = &(new vector());
			_worldVel = &(new vector());
			_worldAcc = &(new vector());
			//Transformation matrix.
			_world_T_local = &(new quaternion());
		}
		//Rotations.
		vector::ptr _localTheta;					//UNUSED.
		vector::ptr _localOmega;
		vector::ptr _localAlpha;					//UNUSED.
		vector::ptr _worldTheta;					//UNUSED -> Same as quaternion.
		vector::ptr _worldOmega;					//UNUSED.
		vector::ptr _worldAlpha;					//UNUSED.
		//Translations.
		vector::ptr _localPos;						//UNUSED.
		vector::ptr _localVel;						//UNUSED.
		vector::ptr _localAcc;
		vector::ptr _worldPos;						//UNUSED.
		vector::ptr _worldVel;						//UNUSED.
		vector::ptr _worldAcc;
		//Transformation matrix.
		quaternion::ptr _world_T_local;
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
		controller(const c_settings::layout, const t_settings::alias, const float=0.04);

		//Setters ******************************************************************
		void assignImu(const MPU6050::cptr&);
		void assignReceiver(const RX::cptr&, const rx_settings::mode);
		void assignPids(const PID::cptr&, const PID::cptr&, const PID::cptr&);
		void assignDrives(const ESC::cptr&, const ESC::cptr&, const ESC::cptr&, const ESC::cptr&);
		void initialize(void);
		void update(void);
		void updateReceiverData(void);
		void updateDesiredAttitude(void);
		void updateSafetyState(void);
		void updateRawData(void);
		void updateLocalData(void);
		void updateActualAttitude(void);
		void updateWorldData(void);
		void updateOutputs(void);
		void driveMotors(void);
		void resetOutputs(void);
		void stopMotors(void);
		void enableMotors(void);
		void disableMotors(void);

		//Getters ******************************************************************
		bool getMovement(const float p);
		float getLooptime(void);

		vector getState(void);
		/**quaternion getAttitude(void);
		vector getAttitude(void);
		vector getOmega(void);
		vector getLocalAcceleration(void);
		vector getLocalVelocity(void);
		vector getGlobalAcceleration(void);
		vector getGlobalCompensatedAcceleration(void);
		vector getGlobalVelocity(void);
		vector getGlobalPosition(void);*/

		int8_t monitorBattery(void);
		int8_t getSafetyState(void);

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
		float _esc1Dc;
		float _esc2Dc;
		float _esc3Dc;
		float _esc4Dc;
		layout _layout;
		safety _safety;
		timer16 _watchdog;		//Watchdog timer for checking calculations time.

		//Quadcopter state vector **************************************************
		state _state;

		//
		vector getFeedbackDc(void);

		void updateActualAttitude(const vector::cptr&, const vector::cptr&, const quaternion::cptr&, quaternion::cptr&);
		bool getMovement(const vector::cptr&, const float);

		//Deprecated ***************************************************************
		const float _maxRoll  = 0.78539816339;
		const float _maxPitch = 0.78539816339;
		const float _maxYaw   = 6.28318530718;
		const quaternion _qI = quaternion(); //Unit quaternion.
		const vector _maxRPS = vector(200.0, 200.0, 200.0);
		float _dbDc;
		float _maxDbDc;
		float _minDbDc;

};
#endif
