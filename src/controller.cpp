/******************************************************************************
 * Quadcopter-Library-v1
 * controller.cpp
 * 
 * TODO::most functions are order dependent. Pass through global var.
 * 
 * @author:	Rob Mertens
 * @date:	14/08/2016
 * @version:	1.1.1
 ******************************************************************************/

#include "controller.h"

/*******************************************************************************
 * Constructor for the controller-class.
 ******************************************************************************/
controller::controller(c_layout layout, t_alias alias, float deadband)
{
	//Layout.
	_layout = layout;

	//Deadband.
	_watchdog = timer16(alias);						//TODO::both 8- and 16-bitness. -> inheritance timer-class.
	
	//Deadband.
	_dbDc = deadband;
	_maxDbDc = 0.5 + 0.5*deadband;
	_minDbDc = 0.5 - 0.5*deadband;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::assignImu(MPU6050 * imu)
{
	_imu = imu;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::assignReceiver(RX * rec, rx_mode mode)
{
	_rec = rec;
	_rec->initialize(mode);							//Should be in controller::initialize().
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::assignPids(PID * pidRoll, PID * pidPitch, PID * pidYaw)
{
	_pidRoll = pidRoll;
	_pidPitch = pidPitch;
	_pidYaw = pidYaw;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::assignDrives(ESC * esc1, ESC * esc2, ESC * esc3, ESC * esc4)
{
	_esc1 = esc1;
	_esc2 = esc2;
	_esc3 = esc3;
	_esc4 = esc4;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::initialize(void)
{
	//SENSOR.
	_imu->initialize();
	
	//RX.
	//_rec->initialize(_rxm);
	
	//PIDs.
	//Nothing.
	
	//ESCs.
	_esc1->arm();								//Stardard settings for 4µs.
	_esc2->arm();
	_esc3->arm();
	_esc4->arm();					
	
	//Timer.
	_watchdog.initialize(t_mode::NORMAL, t_interrupt::OVF);
	_watchdog.setPrescaler(64);
}

/*******************************************************************************
 * Update method of the controller. This method sequentially handles the next
 * functions:
 * (1) Update & process receiver inputs.
 * (2) Update & process sensor inputs.
 * (3) Monitor the quadcopters' safety state.
 * (4) PID feedback control.
 * (5) Drive the motors according to safety state.
 * (6) Additional functions.
 * 
 * TODO::measurement update threading. However, threading cannot be done properly
 * with an arduino I guess -> research.
 ******************************************************************************/
void controller::update(void)
{
	//Update receiver inputs.
	updateReceiverData();
	updateDesiredAttitude();
	
	//Update measurements.
	updateRawData();
	updateLocalData();
	updateActualAttitude();
	updateWorldData();
	
	//Quadcopter safety state.
	updateSafetyState();							//This requires receiver data.
	
	//Flying.
	if(_safety==0x02)
	{
		updateOutputs();
		driveMotors();
	}
	
	//Non-flying.
	if(_safety==0x00)
	{
		resetOutputs();							//Reset PID-controllers.
		stopMotors();
	}
		
	//Get calculations time.
	//Debugging purposes.
	_looptime = _watchdog.getNonResetCount()*0.0625f*64.0f*0.000001f;	//TODO::time calculations in timer class.
	_watchdog.reset();							//This should be calculated at loop start.
}

/*******************************************************************************
 * Method for updating the receiver input. The input is given in duty cycles.
 ******************************************************************************/
void controller::updateReceiverData(void)
{
	float desiredRollRad = 0.0;
	float desiredPitchRad = 0.0;
	float desiredYawRad = 0.0;
	
	//Create some margin for throttle.
	_desiredThrottleDc = _rec->getThrottleChannel();
	
	//RPY angles in duty cycle [%].
	_desiredThetaDc = vector(_rec->getRollChannel(),
				 _rec->getPitchChannel(),
				 _rec->getYawChannel());
	
	//Introduce some deadband.
	if(_desiredThetaDc.y < _minDbDc or _desiredThetaDc.y > _maxDbDc)
	{
		desiredRollRad  = _desiredThetaDc.y*_maxRoll;
	}
	if(_desiredThetaDc.x < _minDbDc or _desiredThetaDc.x > _maxDbDc)
	{
		desiredPitchRad = _desiredThetaDc.x*_maxPitch;
	}
	if(_desiredThetaDc.z < _minDbDc or _desiredThetaDc.z > _maxDbDc)
	{
		desiredYawRad   = _desiredThetaDc.z*_maxYaw;
	}
	
	//RPY angles in radians [rad].
	_desiredThetaRad = vector(desiredRollRad,
				  desiredPitchRad,
				  desiredYawRad);
}

/*******************************************************************************
 * Method for 
 ******************************************************************************/
void controller::updateDesiredAttitude(void)
{
	//Create temporary quaternions.
	quaternion qRoll;
	quaternion qPitch;
	quaternion qYaw;
	
	//Consequtive quaternion rotations.
	qRoll  = quaternion(_desiredThetaRad.y, _unitY);
	qPitch = quaternion(_desiredThetaRad.x, _unitX);
	qYaw   = quaternion(_desiredThetaRad.z, _unitZ);
	
	//Desired quaternion.
	_qDes  = qYaw.cross(qPitch.cross(qRoll));
}

/*******************************************************************************
 * Method for updating the safety state.
 *
 * State | Meaning
 * ------|--------
 *   3   | Nearly non-flying stick position.
 *   2   | Flying.
 *   1   | Nearly flying stick position.
 *   0   | Non-flying.
 *  -1   | Alarm.
 ******************************************************************************/
void controller::updateSafetyState(void)
{
	//Make sure the throttle stick is placed in the left-bottom position.
	if(_safety==0x00 and _desiredThrottleDc < 0.05 and _desiredThetaDc.z < 0.05)
	{
		_safety = 0x01;
	}
	//Make sure the throttle stick is placed back in the centre-bottom position.
	else if(_safety==0x01 and _desiredThrottleDc < 0.05 and _desiredThetaDc.z > 0.45 and _desiredThetaDc.z < 0.55)
	{
		_safety = 0x02;
	}
	//Make sure the throttle stick is placed in the left-bottom corner.
	else if(_safety==0x02 and _desiredThrottleDc < 0.05 and _desiredThetaDc.z > 0.95)
	{
		_safety = 0x03;
	}
	//Make sure the throttle stick is placed back in the centre-bottom position.
	else if(_safety==0x03 and _desiredThrottleDc < 0.05 and _desiredThetaDc.z > 0.45 and _desiredThetaDc.z < 0.55)
	{
		_safety &= 0x00;
	}
	//
	else
	{
		//TODO::alarm state.
	}
}

/*******************************************************************************
 * Method for obtaining all data from the sensors.
 * 
 * TODO::add more sensors.
 ******************************************************************************/
void controller::updateRawData(void)
{
	//Update gyro.
	if(_safety!=0x02)
	{	
		_imu->setGyroscopeBias();	 				//Update bias if not flying.
	}
	_imu->updateGyroscope(&_omegaRaw);
	
	//Update accelero.
	_imu->updateAccelero(&_accelRaw);
	
	//Battery volage.
	//TODO::
}

/*******************************************************************************
 * Method for processing all data from the sensors.
 * 
 * TODO::add more sensors.
 ******************************************************************************/
void controller::updateLocalData(void)
{
	//Update gyro.
	_omegaLocal  = _omegaRaw;
	
	//Update accelero.
	_accelLocal = _accelLocal.multiply(0.9f).sum(_accelRaw.multiply(0.1f));	//Complementary filter for noise.
	
	//Battery volage.
	//TODO::
}

/*******************************************************************************
 * Method for determining the attitude quaternion. Source: "Keeping a Good
 * Attitude: A Quaternion-Based Orientation Filter for IMU's and MARG's".
 ******************************************************************************/
void controller::updateActualAttitude(void)
{
	//Local variable declaration.
	float alpha;								//Linear interpolation complementary filter weight.
	vector worldGravityEst;							//Gavity prediction.
	quaternion qDiff;							
	
	//Gyroscope integration estimation.
	qDiff = _qAtt.cross(quaternion(3.1415f, _omegaLocal));
	_qEst  = _qAtt.sum(qDiff.multiply(0.5*_looptime));			//Integrate rotational velocity with looptime.
	_qEst.norm();
	
	//Accelero gravity vector correction.
	alpha = (float)(!getMovement(1.02));					// 2% determined by testing.
	worldGravityEst  = (_qAtt.conj()).rotate(_accelLocal);			// PREDICTED GRAVITY (Local2World)
	_qCorr = quaternion( sqrt(0.5*(worldGravityEst.z + 1.0)			 ),
			    -worldGravityEst.y/sqrt(2.0*(worldGravityEst.z + 1.0)),
			     worldGravityEst.x/sqrt(2.0*(worldGravityEst.z + 1.0)),
			     0.0f						 );
	_qCorr = (_qI.multiply(1 - alpha)).sum(_qCorr.multiply(alpha));		// LERP
	_qCorr.norm();								// Corrected quaternion.
	
	//Magnetic correction for yaw-angle.
	//Not available on MPU6050.
	//Extra sensor.
	
	//Total quaternion.
	_qAtt = _qEst.cross(_qCorr);
	
	//Euler representation.
	_eulerZXY = _qAtt.q2euler();
}

/*******************************************************************************
 * Method for determining if the quadcopter is moving or not.
 * The total acceleration is compared w/ gravity.
 * 
 * DO NOT USE A GRAVITY CORRECTED ACCELERATION VECTOR.
 *
 * @param: p The user-defired comparision percentage, e.g.: 1,02.
 ******************************************************************************/
bool controller::getMovement(float p)
{
	//Local variables.
	bool movement;
	float eta;
	
	//If acc/g-ratio equals greater than p[%]: dynamic flight.
	//Else: take-off/landing position or hoovering.
	//Sensitivity depends on p, different for bias/quaternion.
	//GRAVITY VALUE DEPENDENT ON SENSOR.
	movement = false;
	eta = abs((_accelLocal.m)/0.91f);					//TODO::determine gravity vector at stand-still, like gyro bias.
	if(eta >= p)
	{	
		movement = true;
	}
	
	//Return movement status.
	return movement;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::updateWorldData()
{
	//World.
	_accelWorld = (_qAtt.conj()).rotate(_accelLocal);
	
	//Compensate for gravity.
	_accelWorldGravityCompensated = _accelWorld;
	_accelWorldGravityCompensated.z -= 0.91f;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::updateOutputs(void)
{
	
	//Calculate PID rotational velocity outputs [rad/s].
	_desiredOmegaRPS  = vector(_pidRoll->calculate(_qAtt.y,  _qDes.y),
				   _pidPitch->calculate(_qAtt.x, _qDes.x),
				   _pidYaw->calculate(_qAtt.z,   _qDes.z));
	
	//Transform back to duty cycle [%].
	_desiredOmegaDc  = vector(_desiredOmegaRPS.x/(_maxRPS.x*3.1415f/180.0f),
				  _desiredOmegaRPS.y/(_maxRPS.y*3.1415f/180.0f),
				  _desiredOmegaRPS.z/(_maxRPS.z*3.1415f/180.0f));
	
	//Do calculations based on structure.
	//TODO::add more structures.
	switch(_layout)
	{
		case(c_layout::CROSS) :
			_esc1Dc = _desiredThrottleDc - _desiredOmegaDc.x + _desiredOmegaDc.y - _desiredOmegaDc.z;
			_esc2Dc = _desiredThrottleDc + _desiredOmegaDc.x + _desiredOmegaDc.y + _desiredOmegaDc.z;
			_esc3Dc = _desiredThrottleDc + _desiredOmegaDc.x - _desiredOmegaDc.y - _desiredOmegaDc.z;
			_esc4Dc = _desiredThrottleDc - _desiredOmegaDc.x - _desiredOmegaDc.y + _desiredOmegaDc.z;
			break;
		
		case(c_layout::PLUS) :
			_esc1Dc = _desiredThrottleDc - _desiredOmegaDc.x - _desiredOmegaDc.z;
			_esc2Dc = _desiredThrottleDc + _desiredOmegaDc.y + _desiredOmegaDc.z;
			_esc3Dc = _desiredThrottleDc + _desiredOmegaDc.x - _desiredOmegaDc.z;
			_esc4Dc = _desiredThrottleDc - _desiredOmegaDc.y + _desiredOmegaDc.z;
			break;
			
		case(c_layout::NONE) :
		default:
			;;
			break;
	}
	
	//Compensate voltage drop.
	//if (battery_voltage < 1240 && battery_voltage > 800)
	//{
	//	_esc1Dc += _esc1Dc * ((1240 - battery_voltage)/(float)3500);
	//	_esc2Dc += _esc2Dc * ((1240 - battery_voltage)/(float)3500);
	//	_esc3Dc += _esc3Dc * ((1240 - battery_voltage)/(float)3500);
	//	_esc4Dc += _esc4Dc * ((1240 - battery_voltage)/(float)3500);
	//}
	
	//Check duty cycle limits.
	if(_esc1Dc > 1.0)_esc1Dc = 1.0;
	else if(_esc1Dc < 0.1)_esc1Dc = 0.1;
	if(_esc2Dc > 1.0)_esc2Dc = 1.0;
	else if(_esc2Dc < 0.1)_esc1Dc = 0.1;
	if(_esc3Dc > 1.0)_esc3Dc = 1.0;
	else if(_esc3Dc < 0.1)_esc3Dc = 0.1;
	if(_esc4Dc > 1.0)_esc4Dc = 1.0;
	else if(_esc4Dc < 0.1)_esc4Dc = 0.1;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::driveMotors(void)
{	
	// Write motor speed.
	_esc1->writeSpeed(_esc1Dc);
	_esc2->writeSpeed(_esc2Dc);
	_esc3->writeSpeed(_esc3Dc);
	_esc4->writeSpeed(_esc4Dc);
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::resetOutputs(void)
{
	_pidRoll->reset();
	_pidPitch->reset();
	_pidYaw->reset();
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::stopMotors(void)
{	
	//Stop the motors at low speed.
	_esc1->writeSpeed(0.0);
	_esc2->writeSpeed(0.0);
	_esc3->writeSpeed(0.0);
	_esc4->writeSpeed(0.0);
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::enableMotors(void)
{	
	//TODO::
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::disableMotors(void)
{	
	//TODO::
}

/*******************************************************************************
 * 
 ******************************************************************************/
float controller::getLooptime(void)
{
	return _looptime;
}

/*******************************************************************************
 * Method for returning the quadcopters state vector.
 ******************************************************************************/
vector controller::getState(void)
{
	//TODO::
	return vector();
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t controller::monitorBattery(void)
{
	int8_t ret = 0;
	
	//if(..bat..)ret = -1;
	
	return ret;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t controller::getSafetyState(void)
{
	return _safety;
}
