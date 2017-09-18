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

using namespace c_settings;
using namespace vmath;

/*******************************************************************************
 * @brief Constructor for the controller-class.
 * @param layout TODO::
 * @param alias
 * @param deadband
 ******************************************************************************/
controller::controller(const c_settings::layout& layout,
											 const t_settings::alias& alias,
											 const float& deadband)
										 : MAX_ANGLE(new vector(0.785f, 0.785f, 6.283f))
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
 * 				(2) Monitor the quadcopters' safety state.
 *				(3) Update & process sensor inputs.
 * 				(4) PID feedback control.
 * 				(5) Drive the motors according to safety state.
 * 				(6) Additional functions.
 * 				TODO::measurement update threading. However, threading cannot be done properly
 * 				with an arduino I guess -> research.
 ******************************************************************************/
void controller::update(void)
{
	//(1) Update receiver inputs.
	updateReceiverData(_rec, _inputs);
	updateDesiredAttitude(_inputs, _desired.local_R_world);

	//(2) Quadcopter safety state.
	updateSafetyState(_inputs, _safety);

	//(3) Update measurements.
	quaternion::ptr world_R_local(new quaternion);
	updateLocalData(_imu, _safety, _actual.localOmega, _actual.localAcc);
	updateActualAttitude(_actual.localOmega, _actual.localAcc, _looptime,
		_actual.local_R_world, _actual.local_R_world);
	conjugate(_actual.local_R_world, world_R_local);
	updateWorldData(_actual.localAccel, world_R_local, _world.worldAcc);

	//(4) PID feedback.
	if(_safety==safety::ON)
	{
		updateOutputs(_pidRoll, _pidPitch, _pidYaw, _inputs, _actual, _desired,
			_layout, _outputs);
	}
	if(_safety==safety::OFF)
	{
		resetOutputs(_pidRoll, _pidPitch, _pidYaw, _outputs);
	}

	//(5) Drive motors according to safety state.
	driveMotors(_outputs, _esc1, _esc2, _esc3, _esc4);

	//(6) Additional functions.
	//Get calculations time.
	//Debugging purposes.
	_looptime = _watchdog.getNonResetCount()*0.0625f*64.0f*0.000001f;	//TODO::time calculations in timer class.
	_watchdog.reset();							//This should be calculated at loop start.
}

/*******************************************************************************
 * @brief Method for
 * @param rec The receiver.
 * @param[out] inputs The receiver input values.
 ******************************************************************************/
 void constroller::updateReceiverData(const rx::cptr& rec,
	 																		inputs& inputs)
 {
	 inputs.throttle = rec->getThrottleInput();
	 inputs.roll 		 = rec->getRollInput();
	 inputs.pitch 	 = rec->getPitchInput();
	 inputs.yaw 		 = rec->getYawInput();
	 //inputs.extra		 = rec->getExtraInput();
 }

/*******************************************************************************
 * @brief
 * @param
 * @param[out]
 ******************************************************************************/
void controller::updateDesiredAttitude(const inputs& inputs,
																			 quaternion::cptr& desired)
{
	//Create temporary quaternions.
	//TODO::to radians/degrees.
	quaternion::ptr roll(new quaternion(inputs.roll,
		vector(0.0f, 1.0f, 0.0f)));
	quaternion::ptr pitch(new quaternion(inputs.pitch,
		vector(1.0f, 0.0f, 0.0f)));
	quaternion::ptr yaw(new quaternion(inputs.yaw,
		vector(0.0f, 0.0f, 1.0f)));

	//Desired quaternion.
	cross(pitch, roll, desired);
	cross(desired, yaw, desired);
}

/*******************************************************************************
 * @brief Method for updating the safety state.
 * @param inputs The receiver inputs.
 * @param[out] safety
 ******************************************************************************/
void controller::updateSafetyState(const inputs& inputs
																	 safety& safety)
{
	switch(safety)
	{
		case safety::OFF :
			//Make sure the throttle stick is placed in the left-bottom position.
			if(inputs.throttle < 0.05 and inputs.yaw < 0.05)safety=safety::T1;
			break;
		case safety::T1 :
			//Make sure the throttle stick is placed back in the centre-bottom position.
			if(inputs.throttle < 0.05 and inputs.yaw > 0.45 and inputs.yaw < 0.55)
				safety=safety::ON;
			break;
		case safety::ON :
			//Make sure the throttle stick is placed in the right-bottom corner.
			if(inputs.throttle < 0.05 and inputs.yaw > 0.95)safety=safety::T2;
			break;
		case safety::T2 :
			//Make sure the throttle stick is placed back in the centre-bottom position.
			if(inputs.throttle < 0.05 and inputs.yaw > 0.45 and inputs.yaw < 0.55)
				safety=safety::OFF;
			break;
		case safety::E1 :
		case default :
			//TODO::
			break;
	}
}

/*******************************************************************************
 * @brief Method for obtaining all data from the sensors.
 * @param safety
 * @param[out] omega
 * @param[out] accel
 ******************************************************************************/
void controller::updateLocalData(const MPU6050::cptr& imu,
																 const safety& safety,
															 	 vector::cptr& omega,
															 	 vector::cptr& accel)
{
	//Complementary filter old accel.
	vector::cptr v(*accel);

	//Update gyroscope bias if not flying.
	if(safety!=state::ON)
	{
		imu->setGyroscopeBias();
	}
	imu->updateGyroscope(omega);
	imu->updateAccelero(accel);

	//Complementary filter new accel.
	vector::cptr w(*accel);

	//Complementary filter for noise.
	//TODO::this is wrong... -> save complemetary filter accel.
	sum(v->multiply(0.9f), w->multiply(0.1f), accel);
}

/*******************************************************************************
 * @brief Method for determining the attitude quaternion.
 *				Source: "Keeping a Good Attitude: A Quaternion-Based Orientation
 * 				Filter for IMU's and MARG's".
 * 				TODO::change to new vmath class.
 * @param localOmega
 * @param localOmega
 * @param input
 * @param[out] output
 ******************************************************************************/
void controller::updateActualAttitude(const vector::cptr& localOmega,
																			const vector::cptr& localAccel,
																			const float time,
																			const quaternion::cptr& input,
																			quaternion::cptr& output)
{
	//Gyroscope integration estimation.
	quaternion::ptr theta(new quaternion);
	quaternion::ptr omega(new quaternion(3.1415f, localOmega));
	quaternion::ptr base(new quaternion);
	cross(input, omega, theta);
	theta->multiply(0.5*time);
	sum(input, theta, base);
	base->normalize();

	//Accelero correction.
	float alpha;
	vector::ptr worldAccelEstimation(new vector));
	quaternion::ptr unit(new quaternion());
	quaternion::ptr conjugate(new quaternion);
	quaternion::ptr correction(new quaternion);
	conjugate(input, conjugate);
	rotate(conjugate, localAccel, *worldAccelEstimation);
	quaternion::ptr correction(quaternion(
		 sqrt(0.5*(worldAccelEstimation->z() + 1.0f)),
		-worldAccelEstimation->y()/sqrt(2.0f*(worldAccelEstimation->z() + 1.0f)),
		 worldAccelEstimation->x()/sqrt(2.0f*(worldAccelEstimation->z() + 1.0f)),
		 0.0f));
	getMovement(localAccel, 1.02f, alpha);															// 2% determined by testing.
	lerp(unit, correction, alpha, correction);
	//slerp(unit, correction, alpha, correction);
	//TODO::fix slerp.
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
													 	 float& eta)
{
	bool movement = false;
	//TODO::determine gravity vector at stand-still, like gyro bias.
	eta = abs((accel->mag())/0.91f);
	if(eta >= p)
	{
		movement = true;
	}
	return movement;
}

/*******************************************************************************
 * @brief TODO::passthrough the struct.
 * @param localAccel
 * @param[out] worldAccel
 ******************************************************************************/
void controller::updateWorldData(const vector::cptr& localAccel,
																 const quaternion::cptr& world_R_local,
																 vector::cptr& worldAccel)
{
	//For now only acceleration is translated to world.
	//TODO::for more vectors, this requires more sensors.
	rotate(world_R_local, localAccel, worldAccel);
}

/*******************************************************************************
 *
 ******************************************************************************/
void controller::updateOutputs(const PID::cptr& roll,
															 const PID::cptr& pitch,
															 const PID::cptr& yaw,
															 const inputs& inputs,
															 const quaternion::cptr& actual,
															 const quaternion::cptr& desired,
															 const layout& layout,
															 outputs& outputs)
{

	//Calculate PID rotational velocity outputs [rad/s].
	//Map to duty cycle [%].
	//TODO::make the map adjustable.
	vector::ptr feedback(new vector(
		pitch->calculate(actual->x(), desired->x())*0.785f*3.1415f/180.0f,
		roll->calculate(actual->y(), desired->y())*0.785f*3.1415f/180.0f,
		yaw->calculate(actual->z(), desired->z())*6.283f*3.1415f/180.0f));

	//Do calculations based on structure.
	switch(layout)
	{
		case layout::CROSS :
			outputs.esc1 = inputs.throttle - feedback.x() + feedback.y() - feedback.z();
			outputs.esc2 = inputs.throttle + feedback.x() + feedback.y() + feedback.z();
			outputs.esc3 = inputs.throttle + feedback.x() - feedback.y() - feedback.z();
			outputs.esc4 = inputs.throttle - feedback.x() - feedback.y() + feedback.z();
			break;

		case layout::PLUS :
			outputs.esc1 = inputs.throttle - feedback.x() - feedback.z();
			outputs.esc2 = inputs.throttle + feedback.y() + feedback.z();
			outputs.esc3 = inputs.throttle + feedback.x() - feedback.z();
			outputs.esc4 = inputs.throttle - feedback.y() + feedback.z();
			break;

		case layout::NONE :
		default :
			;;
			break;
	}

	//Compensate voltage drop.
	//if (battery_voltage < 1240 && battery_voltage > 800)
	//{
	//	outputs.esc1 += outputs.esc1 * ((1240 - battery_voltage)/(float)3500);
	//	outputs.esc2 += outputs.esc2 * ((1240 - battery_voltage)/(float)3500);
	//	outputs.esc3 += outputs.esc3 * ((1240 - battery_voltage)/(float)3500);
	//	outputs.esc4 += outputs.esc4 * ((1240 - battery_voltage)/(float)3500);
	//}

	//Check duty cycle limits.
	if(outputs.esc1 > 1.0)outputs.esc1 = 1.0;
	else if(outputs.esc1 < 0.1)outputs.esc1 = 0.1;
	if(outputs.esc2 > 1.0)outputs.esc2 = 1.0;
	else if(outputs.esc2 < 0.1)outputs.esc1 = 0.1;
	if(outputs.esc3 > 1.0)outputs.esc3 = 1.0;
	else if(outputs.esc3 < 0.1)outputs.esc3 = 0.1;
	if(outputs.esc4 > 1.0)outputs.esc4 = 1.0;
	else if(outputs.esc4 < 0.1)outputs.esc4 = 0.1;
}

/*******************************************************************************
 * @brief
 * @param
 * @param[out]
 * @param[out]
 * @param[out]
 * @param[out]
 ******************************************************************************/
void controller::driveMotors(const outputs& outputs,
														 ESC::cptr& esc1,
													 	 ESC::cptr& esc2,
													 	 ESC::cptr& esc3,
													 	 ESC::cptr& esc4)
{
	esc1->writeSpeed(outputs.esc1);
	esc2->writeSpeed(outputs.esc2);
	esc3->writeSpeed(outputs.esc3);
	esc4->writeSpeed(outputs.esc4);
}

/*******************************************************************************
 *
 ******************************************************************************/
void controller::resetOutputs(PID::cptr& roll,
															PID::cptr& pitch,
															PID::cptr& yaw,
															outputs& outputs)
{
	roll->reset();
	pitch->reset();
	yaw->reset();
	outputs.reset();
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
