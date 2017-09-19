/******************************************************************************
 * Quadcopter-Library-v1
 * controller.cpp
 *
 * TODO::most functions are order dependent. Pass through global var.
 * TODO::make this a namespace -> at this way the controller functions end up
 *			 in the .ino and the user has "more" control.
 *
 * @author:	Rob Mertens
 * @date:	14/08/2016
 * @version:	1.1.1
 ******************************************************************************/

#include "controller.h"

using namespace vmath;

namespace controller
{
	/*****************************************************************************
	 * @brief Method for
	 * @param rec The receiver.
	 * @param[out] inputs The receiver input values.
	 ****************************************************************************/
	 void updateReceiverInputs(const RX::cptr& receiver,
		 												 inputs& inputs)
	 {
		 inputs.throttle = receiver->getThrottleInput();
		 inputs.roll 		 = receiver->getRollInput();
		 inputs.pitch 	 = receiver->getPitchInput();
		 inputs.yaw 		 = receiver->getYawInput();
	 }

	/*****************************************************************************
	 * @brief
	 * @param
	 * @param[out]
	 ****************************************************************************/
	void updateDesiredState(const inputs& inputs,
													state& desired)
	{
		//Create temporary quaternions.
		//TODO::to radians/degrees.
		quaternion::ptr roll(new quaternion(_UNITY, inputs.roll));
		quaternion::ptr pitch(new quaternion(_UNITX, inputs.pitch));
		quaternion::ptr yaw(new quaternion(_UNITZ,inputs.yaw));

		//Desired quaternion.
		cross(pitch, roll, desired.local_R_world);
		cross(desired.local_R_world, yaw, desired.local_R_world);
	}

	/*******************************************************************************
	 * @brief Method for updating the safety state.
	 * @param inputs The receiver inputs.
	 * @param[out] safety
	 ******************************************************************************/
	void updateSafetyState(const inputs& inputs,
																		 safety& safety)
	{
		switch(safety)
		{
			case c_settings::safety::OFF :
				//Make sure the throttle stick is placed in the left-bottom position.
				if(inputs.throttle < _DEADBAND and inputs.yaw < _DEADBAND)
					safety=c_settings::safety::T1;
				break;
			case c_settings::safety::T1 :
				//Make sure the throttle stick is placed back in the centre-bottom position.
				if(inputs.throttle < _DEADBAND and inputs.yaw > (0.5f-_DEADBAND) and
						inputs.yaw < (0.5f+_DEADBAND))
					safety=c_settings::safety::ON;
				break;
			case c_settings::safety::ON :
				//Make sure the throttle stick is placed in the right-bottom corner.
				if(inputs.throttle < _DEADBAND and inputs.yaw > (1.0f-_DEADBAND))
					safety=c_settings::safety::T2;
				break;
			case c_settings::safety::T2 :
				//Make sure the throttle stick is placed back in the centre-bottom position.
				if(inputs.throttle < _DEADBAND and inputs.yaw > (0.5f-_DEADBAND) and
						inputs.yaw < (0.5f+_DEADBAND))
					safety=c_settings::safety::OFF;
				break;
			case c_settings::safety::E1 :
			default :
				//TODO::
				break;
		}
	}

	/*****************************************************************************
	 * @brief Method for obtaining all data from the sensors.
	 * @param safety
	 * @param[out] omega
	 * @param[out] accel
	 ****************************************************************************/
	void updateActualState(const MPU6050::cptr& imu,
												 const float time,
											 	 state& actual)
	{
		//Update first vectors in local frame.
		updateLocalData(imu, actual);

		//Update the attitude quaternion.
		updateActualAttitude(time, actual);

		//Update the world vectors with the new attitude quaternion.
		quaternion::ptr world_R_local(new quaternion);
		conjugate(actual.local_R_world, world_R_local);
		updateWorldData(actual.localAccel, world_R_local, actual.worldAccel);
	}

	/*****************************************************************************
	 * @brief Method for obtaining all data from the sensors.
	 * @param safety
	 * @param[out] omega
	 * @param[out] accel
	 ****************************************************************************/
	void updateLocalData(const MPU6050::cptr& imu,
											 state& actual)
	{
		//Complementary filter inputs.
		vector::cptr v(new vector);
		vector::cptr w(new vector);
		*v = *actual.localAccel;

		//Update IMU readings.
		imu->updateGyroscope(actual.localOmega);
		imu->updateAccelero(actual.localAccel);

		//Update gyroscope bias if not moving.
		if(!getMovement(actual.localAccel, 1.02f))imu->setGyroscopeBias();

		//Complementary filter new accel.
		*w = *actual.localAccel;
		v->multiply(0.9f);
		w->multiply(0.1f);

		//Complementary filter for noise.
		//TODO::this is wrong... -> save complemetary filter accel.
		sum(v, w, actual.localAccel);
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
	void updateActualAttitude(const float time,
														state& actual)
	{
		//Save actual state in const.
		const state before = actual;

		//Gyroscope integration estimation.
		quaternion::ptr theta(new quaternion);
		quaternion::ptr omega(new quaternion(before.localOmega, 3.1415f));
		quaternion::ptr base(new quaternion);
		cross(before.local_R_world, omega, theta);
		theta->multiply(0.5*time);
		sum(before.local_R_world, theta, base);
		base->normalize();

		//Accelero correction.
		float alpha;
		vector::ptr worldAccelEstimation(new vector);
		quaternion::ptr conjugated(new quaternion);
		conjugate(before.local_R_world, conjugated);
		rotate(conjugated, before.localAccel, worldAccelEstimation);
		quaternion::ptr correction(new quaternion(
			 sqrt(0.5*(worldAccelEstimation->z() + 1.0f)),
			-worldAccelEstimation->y()/sqrt(2.0f*(worldAccelEstimation->z() + 1.0f)),
			 worldAccelEstimation->x()/sqrt(2.0f*(worldAccelEstimation->z() + 1.0f)),
			 0.0f));
		getMovement(before.localAccel, 1.02f, alpha);																// 2% determined by testing.
		lerp(_REAL, correction, alpha, correction);
		//slerp(_REAL, correction, alpha, correction);
		//TODO::fix slerp.
		correction->normalize();

		//Magnetic correction for yaw-angle.
		//Not available on MPU6050.
		//Extra sensor.

		//Total quaternion.
		cross(base, correction, actual.local_R_world);
		actual.local_R_world->normalize();
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
	bool getMovement(const vector::cptr& accel,
									 const float p,
								 	 float& eta)
	{
		bool movement = false;
		//TODO::determine gravity vector at stand-still, like gyro bias.
		eta = abs((accel->magnitude())/0.91f);
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
	void updateWorldData(state& actual)
	{
		//For now only acceleration is translated to world.
		//TODO::for more vectors, this requires more sensors.
		rotate(actual.world_R_local, actual.localAccel, actual.worldAccel);
	}

	/*******************************************************************************
	 *
	 ******************************************************************************/
	void updateOutputs(const PID::cptr& roll,
										 const PID::cptr& pitch,
										 const PID::cptr& yaw,
										 const inputs& inputs,
										 const state& actual,
										 const state& desired,
										 const layout& layout,
										 outputs& outputs)
	{

		//Calculate PID rotational velocity outputs [rad/s].
		//Map to duty cycle [%].
		//TODO::make the map adjustable.
		vector::ptr feedback(new vector(
			pitch->calculate(actual->x(), desired->x())*_MAXANGLE->x(),
			roll->calculate(actual->y(), desired->y())*_MAXANGLE->y(),
			yaw->calculate(actual->z(), desired->z())*_MAXANGLE->z()));

		//Do calculations based on structure.
		switch(layout)
		{
			case c_settings::layout::CROSS :
				outputs.esc1 = inputs.throttle - feedback->x() + feedback->y() - feedback->z();
				outputs.esc2 = inputs.throttle + feedback->x() + feedback->y() + feedback->z();
				outputs.esc3 = inputs.throttle + feedback->x() - feedback->y() - feedback->z();
				outputs.esc4 = inputs.throttle - feedback->x() - feedback->y() + feedback->z();
				break;
			case c_settings::layout::PLUS :
				outputs.esc1 = inputs.throttle - feedback->x() - feedback->z();
				outputs.esc2 = inputs.throttle + feedback->y() + feedback->z();
				outputs.esc3 = inputs.throttle + feedback->x() - feedback->z();
				outputs.esc4 = inputs.throttle - feedback->y() + feedback->z();
				break;
			case c_settings::layout::NONE :
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
	void controller::driveMotors(outputs& outputs,
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

}; //End namespace controller.
