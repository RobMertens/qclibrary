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
 * @brief Constructor for the controller-class.
 * @param layout TODO::
 * @param alias
 * @param deadband
 ******************************************************************************/
controller::controller(const c_settings::layout& layout,
											 const t_settings::alias& alias,
											 const float& deadband)
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
 * @brief
 * @param imu
 ******************************************************************************/
void controller::assignImu(const MPU6050::cptr& imu)
{
	_imu = imu;
}

/*******************************************************************************
 * @brief
 * @param rec
 * @param mode
 ******************************************************************************/
void controller::assignReceiver(const RX::cptr& rec,
																const rx_settings::mode mode)
{
	_rec = rec;
	_rec->initialize(mode);							//Should be in controller::initialize().
}

/*******************************************************************************
 * @brief
 * @param pidRoll
 * @param pidPitch
 * @param pidYaw
 ******************************************************************************/
void controller::assignPids(const PID::cptr& pidRoll,
														const PID::cptr& pidPitch,
														const PID::cptr& pidYaw)
{
	_pidRoll = pidRoll;
	_pidPitch = pidPitch;
	_pidYaw = pidYaw;
}

/*******************************************************************************
 * @brief
 * @param esc1
 * @param esc2
 * @param esc3
 * @param esc4
 ******************************************************************************/
void controller::assignDrives(const ESC::cptr& esc1,
															const ESC::cptr& esc2,
															const ESC::cptr& esc3,
															const ESC::cptr& esc4)
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
	_watchdog.initialize(t_settings::mode::NORMAL, t_settings::interrupt::OVF);
	_watchdog.setPrescaler(64);
}

/*******************************************************************************
 * @brief Update method of the controller. This method sequentially handles the next
 * 				functions:
 * 				(1) Update & process receiver inputs.
 * 				(2) Update & process sensor inputs.
 * 				(3) Monitor the quadcopters' safety state.
 * 				(4) PID feedback control.
 * 				(5) Drive the motors according to safety state.
 * 				(6) Additional functions.
 * 				TODO::measurement update threading. However, threading cannot be done properly
 * 				with an arduino I guess -> research.
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
 * @brief Method for updating the receiver input.
 * 				The input is given in duty cycles.
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
	if(_desiredThetaDc.y() < _minDbDc or _desiredThetaDc.y() > _maxDbDc)
	{
		desiredRollRad  = _desiredThetaDc.y()*_maxRoll;
	}
	if(_desiredThetaDc.x() < _minDbDc or _desiredThetaDc.x() > _maxDbDc)
	{
		desiredPitchRad = _desiredThetaDc.x()*_maxPitch;
	}
	if(_desiredThetaDc.z() < _minDbDc or _desiredThetaDc.z() > _maxDbDc)
	{
		desiredYawRad   = _desiredThetaDc.z()*_maxYaw;
	}

	//RPY angles in radians [rad].
	_desiredThetaRad = vector(desiredRollRad, desiredPitchRad,desiredYawRad);
}

/*******************************************************************************
 * Method for
 ******************************************************************************/
void controller::updateDesiredAttitude(void)
{
	//Create temporary quaternions.
	quaternion roll;
	quaternion pitch;
	quaternion yaw;

	//Consequtive quaternion rotations.
	qRoll  = quaternion(_desiredThetaRad.y(), _unitY);
	qPitch = quaternion(_desiredThetaRad.x(), _unitX);
	qYaw   = quaternion(_desiredThetaRad.z(), _unitZ);

	//Desired quaternion.
	_qDes  = qYaw.cross(qPitch.cross(qRoll));
}

/*******************************************************************************
 * @brief Method for updating the safety state.
 * 				State | Meaning
 * 				------|--------
 * 				  3   | Nearly non-flying stick position.
 * 				  2   | Flying.
 * 				  1   | Nearly flying stick position.
 * 				  0   | Non-flying.
 * 				 -1   | Alarm.
 * 				TODO::make deadband values adjustable.
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
 * @brief Method for obtaining all data from the sensors.
 * 				TODO::add more sensors.
 ******************************************************************************/
void controller::updateRawData(vector::cptr& omega,
															 vector::cptr& accel)
{
	if(_safety!=0x02)
	{
		_imu->setGyroscopeBias();	 				//Update bias if not flying.
	}
	_imu->updateGyroscope(omega);
	_imu->updateAccelero(accel);
	//Battery volage.
	//TODO::
}

/*******************************************************************************
 * @brief Method for processing all data from the sensors.
 * 				TODO::add more sensors.
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
 * @brief Method for determining the attitude quaternion.
 *				Source: "Keeping a Good Attitude: A Quaternion-Based Orientation
 * 				Filter for IMU's and MARG's".
 * 				TODO::change to new vmath class.
 * @param omegaLocal
 * @param omegaLocal
 * @param input
 * @param[out] output
 ******************************************************************************/
void controller::updateActualAttitude(const vector::cptr& omegaLocal,
																			const vector::cptr& accelLocal,
																			const float time,
																			const quaternion::cptr& input,
																			quaternion::cptr& output)
{
	//Gyroscope integration estimation.
	quaternion::ptr theta(new quaternion);
	quaternion::ptr omega(new quaternion(3.1415f, omegaLocal));
	quaternion::ptr base(new quaternion);
	cross(input, omega, theta);
	theta->multiply(0.5*time);
	sum(input, theta, base);
	base->normalize();

	//Accelero correction.
	float alpha;
	vector::ptr accelWorldEstimation(new vector));
	quaternion::ptr unit(new quaternion());
	quaternion::ptr conjugate(new quaternion);
	quaternion::ptr correction(new quaternion);
	conjugate(input, conjugate);
	rotate(conjugate, accelLocal, *accelWorldEstimation);
	vector::ptr correction(quaternion(sqrt(0.5*(accelWorldEstimation->_z + 1.0)),
		-accelWorldEstimation->y/sqrt(2.0*(accelWorldEstimation->_z + 1.0)),
		 accelWorldEstimation->x/sqrt(2.0*(accelWorldEstimation->_z + 1.0)), 0.0f));
	alpha = (float)(!getMovement(accelLocal, 1.02f));															// 2% determined by testing.
	lerp(unit, correction, alpha, correction);
	//slerp(unit, correction, alpha, correction);
	correction->normalize();

	//Magnetic correction for yaw-angle.
	//Not available on MPU6050.
	//Extra sensor.

	//Total quaternion.
	cross(base, correction, output);
	output->normalize();
}

/*******************************************************************************
 * @brief Method for determining if the quadcopter is moving or not.
 * 				The total acceleration is compared w/ gravity.
 * 				DO NOT USE A GRAVITY CORRECTED ACCELERATION VECTOR.
 * 				If acc/g-ratio equals greater than p[%]: dynamic flight.
 * 				Else: take-off/landing position or hoovering.
 * 				Sensitivity depends on p, different for bias/quaternion.
 * 				GRAVITY VALUE DEPENDENT ON SENSOR.
 * @param accel
 * @param p The user-defired comparision percentage, e.g.: 1,02.
 * @param[out] eta The linear value expressing movement.
 ******************************************************************************/
bool controller::getMovement(const vector::cptr& accel,
														 const float p,
													 	 float eta&)
{
	bool movement = false;
	eta = abs((accel->mag())/0.91f);					//TODO::determine gravity vector at stand-still, like gyro bias.
	if(eta >= p)
	{
		movement = true;
	}
	return movement;
}

/*******************************************************************************
 *
 ******************************************************************************/
void controller::updateWorldData(void)
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
		case(c_settings::layout::CROSS) :
			_esc1Dc = _desiredThrottleDc - _desiredOmegaDc.x + _desiredOmegaDc.y - _desiredOmegaDc.z;
			_esc2Dc = _desiredThrottleDc + _desiredOmegaDc.x + _desiredOmegaDc.y + _desiredOmegaDc.z;
			_esc3Dc = _desiredThrottleDc + _desiredOmegaDc.x - _desiredOmegaDc.y - _desiredOmegaDc.z;
			_esc4Dc = _desiredThrottleDc - _desiredOmegaDc.x - _desiredOmegaDc.y + _desiredOmegaDc.z;
			break;

		case(c_settings::layout::PLUS) :
			_esc1Dc = _desiredThrottleDc - _desiredOmegaDc.x - _desiredOmegaDc.z;
			_esc2Dc = _desiredThrottleDc + _desiredOmegaDc.y + _desiredOmegaDc.z;
			_esc3Dc = _desiredThrottleDc + _desiredOmegaDc.x - _desiredOmegaDc.z;
			_esc4Dc = _desiredThrottleDc - _desiredOmegaDc.y + _desiredOmegaDc.z;
			break;

		case(c_settings::layout::NONE) :
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
 * @brief
 * @return
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
