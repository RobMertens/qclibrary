#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdint.h>
#include "math.h"
#include "settings.h"
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
		controller(c_layout, float=0.04);

		//Setters ********************************************************************
		void update(void);
		void measure(void);
		void receive(void);
		void enableMotors(void);
		void driveMotors(void);
		void disableMotors(void);
		
		//Getters ********************************************************************
		float getLooptime(void);
		vector getState(void);
		quaternion getAttitude(void);
		vector getAttitude(void);
		vector getOmega(void);
		vector getLocalAcceleration(void);
		vector getLocalVelocity(void);
		vector getGlobalAcceleration(void);
		vector getGlobalCompensatedAcceleration(void);
		vector getGlobalVelocity(void);
		vector getGlobalPosition(void);
		
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
		
		MPU6050 * _imu;
		
		timer16 _watchdog;		//Watchdog timer for checking calculations time.
		
		float _looptime;
		
		// State vectors
		static const vector _unitX = vector(1.0, 0.0, 0.0);
		static const vector _unitY = vector(0.0, 1.0, 0.0);
		static const vector _unitZ = vector(0.0, 0.0, 1.0);
			
		vector _thetaLocal; 		//True angles w.r.t. Euler by integrating factors.
		vector _omegaLocal;
		
		vector _accelLocal;
		vector _accelWorld;
		vector _accelWorldGravityCompensated;
		
		vector _eulerZXY; 		//RPY angles.
		quaternion _qAtt;
		
		static const float _maxRoll  = 0.78539816339;
		static const float _maxPitch = 0.78539816339;
		static const float _maxYaw   = 6.28318530718;
		// Config.
		c_layout _layout;
		
		// Safety state.
		// State | Meaning
		// ------|--------
		//   3   | Nearly non-flying stick position.
		//   2   | Flying.
		//   1   | Nearly flying stick position.
		//   0   | Non-flying.
		//  -1   | Alarm.
		int8_t _safety;
		
		// Control.
		float _dcDesiredThrottle;
		float _dcDesiredRoll;
		float _dcDesiredPitch;
		float _dcDesiredYaw;
		
		// Control.
		float _dcOutputThrottle;
		float _dcOutputRoll;
		float _dcOutputPitch;
		float _dcOutputYaw;
		
		float _dcEsc1;
		float _dcEsc2;
		float _dcEsc3;
		float _dcEsc4;
		
		float _dcDeadband;
		float _dcDeadbandLowerBoundary;
		float _dcDeadbandUpperBoundary;
		
		//
		vector getFeedbackDc(void);
		
};
#endif

