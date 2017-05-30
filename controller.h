#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdint.h>
#include "math.h"
#include "settings.h"
#include "timer8.h"
#include "timer16.h"
#include "src/vmath.h"
#include "src/MPU6050.h"
#include "src/PID.h"
#include "src/RX.h"
#include "src/ESC.h"
#include "src/LED.h"

enum class c_mode : uint8_t
{
	NONE	= 0,
	ATT	= 1,
	VEL	= 2,	// GPS Needed.
	POS	= 3,	// GPS Needed.
	HVR	= 4
}

enum class c_layout : uint8_t
{
	NONE	= 0,
	PLUS	= 1,
	CROSS	= 2
}

class controller
{
	public:
		//Constructors ***************************************************************
		controller(MPU6050, RX, PID, PID, PID, ESC, ESC, ESC, ESC, c_layout, t_alias, float=0.04);

		//Setters ********************************************************************
		void initialize(void);
		void update(void);
		
		void updateReceiverData(void);
		void updateDesiredAttitude(void);
		void updateSafetyState(void);
		void updateRawData(void)
		void updateLocalData(void)
		void updateActualAttitude(void)
		void updateWorldData();
		void updateOutputs(void);
		void driveMotors(void);
		void resetOutputs(void);
		void stopMotors(void);
		void enableMotors(void);
		void disableMotors(void)
		
		//Getters ********************************************************************
		bool getMovement(float p)
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
		
		int8_t monitorBattery(void)
		int8_t getSafetyState(void)
		
	private:
		// Hardware components
		ESC * _esc1;
		ESC * _esc2;
		ESC * _esc3;
		ESC * _esc4;
		
		PID * _pidRoll;
		PID * _pidPitch;
		PID * _pidYaw;
		
		RX * _rec;
		rx_mode _rxm;
		
		MPU6050 * _imu;
		
		timer16 _watchdog;		//Watchdog timer for checking calculations time.
		
		float _looptime;
		
		// State vectors
		static const vector _unitX = vector(1.0, 0.0, 0.0);
		static const vector _unitY = vector(0.0, 1.0, 0.0);
		static const vector _unitZ = vector(0.0, 0.0, 1.0);
		
		vector _omegaRaw;
		vector _accelRaw;
		
		vector _omegaLocal;
		vector _accelLocal;
		
		quaternion _qDes;		//Desired attitude/orientation quaternion.
		quaternion _qAtt;		//Actual attitude/orientation quaternion.
		quaternion _qEst;		//Estimated orientation quaternion.
		quaternion _qCorr;		//Correction quaternion.
		vector _eulerZXY; 		//RPY angles.
		
		vector _accelWorld;
		vector _accelWorldGravityCompensated;
		
		static const float _maxRoll  = 0.78539816339;
		static const float _maxPitch = 0.78539816339;
		static const float _maxYaw   = 6.28318530718;
		static const quaternion _qI = quaternion(); //Unit quaternion.
		static const vector _maxRPS = vector(200.0, 200.0, 200.0);
		
		// Config.
		c_layout _layout;
		
		// Safety state.
		int8_t _safety;
		
		// Control.
		float  _desiredThrottleDc;
		vector _desiredThetaDc();
		vector _desiredThetaRad();
		vector _desiredOmegaRPS();
		vector _desiredOmegaDc();
				
		float _esc1Dc;
		float _esc2Dc;
		float _esc3Dc;
		float _esc4Dc;
		
		float _dbDc;
		float _maxDbDc;
		float _minDbDc;
		
		//
		vector getFeedbackDc(void);
		
		void setQuaternion(quaternion *, vector *, vector *);
		bool getMovement(vector *, float);
		
};
#endif

