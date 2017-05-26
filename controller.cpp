/******************************************************************************
 * Quadcopter-Library-v1
 * controller.cpp
 *
 *	       TOP-VIEW			     BOTTOM-VIEW
 *
 *		  y				  y
 *		  |				  |
 *
 * 	/ 2 \   front   / 1 \		/ 2 \   front   / 1 \
 * 	\cw /           \ccw/		\ccw/           \cw /
 * 	   |::.........::|   		   |::.........::|   
 * 	 l   |:       :|   r		 l   |:       :|   r
 * 	 e    |:     :|    i		 e    |:     :|    i
 * 	 f     |::+::|     g	-x	 f     |::+::|     g	-x
 * 	 t    |:     :|    h		 t    |:     :|    h
 * 	     |:       :|   t		     |:       :|   t
 * 	   |::.........::|		   |::.........::|
 * 	/ 3 \           / 4 \		/ 3 \           / 4 \
 * 	\ccw/   rear    \cw /		\cw /   rear    \ccw/
 *
 * @author:	Rob Mertens
 * @date:	14/08/2016
 * @version:	1.1.1
 ******************************************************************************/

#include "controller.h"

/*******************************************************************************
 * Constructor for the controller-class.
 ******************************************************************************/
controller::controller(c_mode mode, c_layout layout)
{	
	
}

/*******************************************************************************
 * 
 ******************************************************************************/
controller::update()
{
	// Init.
	_led.set()
	
	// Update measurements.
	measure();
	
	// Update receiver inputs.
	receive();
	
	// Flying.
	enableMotors();
	driveMotors();
	
	// Non-flying.
	disableMotors();
	
}

/*******************************************************************************
 * 
 ******************************************************************************/
controller::measure()
{
	//Update accelero.
	_imu.setAccelero(&_rawAccel);
	_bodyAccel = _bodyAccel.multiply(0.9f).sum(_rawAccel.multiply(0.1f));
	
	//Update gyro.
	if (!_imu.getMovement(&_bodyAccel, 1.02))_imu.setBias(&_rawBias); 	//Update bias if not moving.
	_imu.setGyroscope(&_rawOmega, &_rawBias);
	_bodyOmega  = _rawOmega;
	
	//Update rotation quaternion.
	_imu.setQuaternion(&_rotQ, &_bodyOmega, &_bodyAccel);
	
	//Update world vectors.
	_worldAccel = (_rotQ.conj()).rotate(_bodyAccel);
	_worldAccel.z -= 0.91f;
	
	//Battery volage.
	//TODO::
}

/*******************************************************************************
 * 
 ******************************************************************************/
controller::receive()
{
	//Receiver channels.
	_dcDesiredThrottle = _rec.getThrottleChannel();
	_dcDesiredRoll     = _rec.getRollChannel();
	_dcDesiredPitch    = _rec.getPitchChannel();
	_dcDesiredYaw      = _rec.getYawChannel();
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::enableMotors()
{
	// Make sure the throttle stick is placed in the left-bottom position.
	if(_rec.getThrottleChannel() < 0.05 and _rec.getYawChannel() < 0.05)
	{
		_safety = 0x01;
	}
	
	// Make sure the throttle stick is placed back in the centre-bottom position.
	if(_safety==0x01 and _rec.getThrottleChannel() < 0.05 and _rec.getYawChannel() > 0.45 and _rec.getYawChannel() < 0.55)
	{
		// Increase safety state.
		_safety = 0x02;
		
		// Reset some variables before flying.
		_led.reset();
		_pid.reset();
		
		// Start the motors at low speed.
		_esc1.writeSpeed(0.1);
		_esc2.writeSpeed(0.1);
		_esc3.writeSpeed(0.1);
		_esc4.writeSpeed(0.1);
	}
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::disableMotors()
{	
	// Make sure the throttle stick is placed in the left-bottom corner.
	if(_safety==0x02 and _rec.getThrottleChannel() < 0.05 and _rec.getYawChannel() > 0.95)
	{
		_safety = 0x03;
	}
	
	// Make sure the throttle stick is placed back in the centre-bottom position.
	if(_safety==0x03 and _rec.getThrottleChannel() < 0.05 and _rec.getYawChannel() > 0.45 and _rec.getYawChannel() < 0.55)
	{
		// Reset safety state.
		_safety &= 0x00;
		
		// Stop the motors.
		_esc1.writeSpeed(0.0);
		_esc2.writeSpeed(0.0);
		_esc3.writeSpeed(0.0);
		_esc4.writeSpeed(0.0);
	}
}

/*******************************************************************************
 * 
 ******************************************************************************/
vector controller::driveMotors()
{
	vector dcOutput();
	
	float dcEsc1;
	float dcEsc2;
	float dcEsc3;
	float dcEsc4;
	
	// Only do calculations when flying.
	if (_safety==0x02)
	{
		//PID feedback output in duty cycle.
		dcOutput = getFeedbackDc();
		
		//TODO::Map max throttle to 80% instead of 100%.
		if (_dcDesiredThrottle > 0.8)_dcDesiredThrottle=0.8;
		
		//Do calculations based on structure.
		//TODO::add more structures.
		dcEsc1 = _dcDesiredThrottle - dcOutput.x + dcOutput.y - dcOutput.z;
		dcEsc2 = _dcDesiredThrottle + dcOutput.x + dcOutput.y + dcOutput.z;
		dcEsc3 = _dcDesiredThrottle + dcOutput.x - dcOutput.y - dcOutput.z;
		dcEsc4 = _dcDesiredThrottle - dcOutput.x - dcOutput.y + dcOutput.z;
		
		//Compensate voltage drop.
		//if (battery_voltage < 1240 && battery_voltage > 800)
		//{
		//	dcEsc1 += dcEsc1 * ((1240 - battery_voltage)/(float)3500);
		//	dcEsc2 += dcEsc2 * ((1240 - battery_voltage)/(float)3500);
		//	dcEsc3 += dcEsc3 * ((1240 - battery_voltage)/(float)3500);
		//	dcEsc4 += dcEsc4 * ((1240 - battery_voltage)/(float)3500);
		//}
		
		//Check duty cycle limits.
		if(dcEsc1 > 1.0)dcEsc1 = 1.0;
		if(dcEsc2 > 1.0)dcEsc2 = 1.0;
		if(dcEsc3 > 1.0)dcEsc3 = 1.0;
		if(dcEsc4 > 1.0)dcEsc4 = 1.0;
		if(dcEsc1 < 0.1)dcEsc1 = 0.1;
		if(dcEsc2 < 0.1)dcEsc1 = 0.1;
		if(dcEsc3 < 0.1)dcEsc3 = 0.1;
		if(dcEsc4 < 0.1)dcEsc4 = 0.1;
		
		// Write motor speed.
		_esc1.writeSpeed(dcEsc1);
		_esc2.writeSpeed(dcEsc2);
		_esc3.writeSpeed(dcEsc3);
		_esc4.writeSpeed(dcEsc4);
	}
}

/*******************************************************************************
 * 
 ******************************************************************************/
vector controller::getFeedbackDc()
{
	// Allocate memory.
	float roll  = 0.0;
	float pitch = 0.0;
	float yaw   = 0.0;
	
	quaternion qRoll();
	quaternion qPitch();
	quaternion qYaw();
	quaternion qDes();
	
	// Calculate desired rotation.
	// Introduce a little deadband for sensivity.
	if(_dcDesiredRoll < 0.48 or _dcDesiredRoll > 0.52)roll = _dcDesiredRoll*_maxRoll;
	if(_dcDesiredPitch < 0.48 or _dcDesiredPitch > 0.52)pitch = _dcDesiredPitch*_maxPitch;
	if(_dcDesiredYaw < 0.48 or _dcDesiredYaw > 0.52)yaw = _dcDesiredYaw*_maxYaw;
	
	// Consequtive quaternion rotations.
	qRoll  = quaterion(roll,  _unitY);
	qPitch = quaterion(pitch, _unitX);
	qYaw   = quaterion(yaw,   _unitZ);
	qDes   = qYaw.cross(qPitch.cross(qRoll));
	
	// Calculate PID rotational velocity outputs [rad/s].
	float dRoll  = _pidRoll(_qAtt.y,  qDes.y);
	float dPitch = _pidPitch(_qAtt.x, qDes.x);
	float dYaw   = _pidYaw(_qAtt.z,   qDes.z);
	
	// Transform back to duty cycle [%].
	float dcRoll  = dRoll/(DPS_MAX*pi/180);
	float dcPitch = dPitch/(DPS_MAX*pi/180);
	float dcYaw   = dYaw/(DPS_MAX*pi/180);
	
	return vector(dcPitch, dcRoll, dcYaw);
}

/*******************************************************************************
 * Method for returning the quadcopters state vector.
 * 
 * 
 ******************************************************************************/
vector controller::getState()
{
	//TODO::
	//return ;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t controller::getSafetyState()
{
	return _safety;
}
