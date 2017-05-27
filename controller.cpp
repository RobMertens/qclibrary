/******************************************************************************
 * Quadcopter-Library-v1
 * controller.cpp
 *
 ******************************************************************************
 *	       TOP-VIEW				    BOTTOM-VIEW
 *
 *	y_plus	  y_cross    x_plus     	y_plus	  y_cross    x_plus
 *      \ 	  |	    /			\ 	  |	    /
 *
 * 	/ 2 \   front   / 1 \			/ 2 \   front   / 1 \
 * 	\cw /           \ccw/			\ccw/           \cw /
 * 	   |::.........::|   			   |::.........::|   
 * 	 l   |:       :|   r			 l   |:       :|   r
 * 	 e    |:     :|    i			 e    |:     :|    i
 * 	 f     |::+::|     g	-x_cross	 f     |::+::|     g	-x_cross
 * 	 t    |:     :|    h			 t    |:     :|    h
 * 	     |:       :|   t			     |:       :|   t
 * 	   |::.........::|			   |::.........::|
 * 	/ 3 \           / 4 \			/ 3 \           / 4 \
 * 	\ccw/   rear    \cw /			\cw /   rear    \ccw/
 * 
 *		
 ******************************************************************************
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
	//ESCs.
	*_esc1 = ESC(t_alias::T1, t_channel::B, 4000, 2000, 700);
	*_esc2 = ESC(t_alias::T3, t_channel::B, 4000, 2000, 700);
	*_esc3 = ESC(t_alias::T1, t_channel::C, 4000, 2000, 700);
	*_esc4 = ESC(t_alias::T3, t_channel::C, 4000, 2000, 700);
	
	//Deadband.
	_dcDeadband = deadband;
	_dcDeadbandUpperBoundary = 0.5 - 0.5*_dcDeadBand;
	_dcDeadbandLowerBoundary = 0.5 + 0.5*_dcDeadBand;

	//Deadband.
	_watchdog = timer16(alias);
}

/*******************************************************************************
 * Constructor for the controller-class.
 ******************************************************************************/
controller::controller(MPU6050 imu, RX rec, PID pidRoll, PID pidPitch, PID pidYaw, ESC esc1, ESC esc2, ESC esc3, ESC esc4, c_layout layout, t_alias alias, float deadband)
{
	//MPU.
	_imu = &imu;
	
	//REC.
	_rec = &rec;
	
	//PID.
	_pidRoll  = &pidRoll;
	_pidPitch = &pidPitch;
	_pidYaw   = &pidYaw;
	
	//ESCs.
	_esc1 = &esc1;
	_esc2 = &esc2;
	_esc3 = &esc3;
	_esc4 = &esc4;
	
	//Deadband.
	_dcDeadband = deadband;
	_dcDeadbandUpperBoundary = 0.5 - 0.5*_dcDeadBand;
	_dcDeadbandLowerBoundary = 0.5 + 0.5*_dcDeadBand;

	//Deadband.
	_watchdog = timer16(alias);
}

/*******************************************************************************
 * Constructor for the controller-class.
 ******************************************************************************/
controller::initialize()
{
	//ESCs.
	_esc1->arm();					//Stardard settings for 4µs.
	_esc2->arm();
	_esc3->arm();
	_esc4->arm();					
	
	//Timer.
	_watchdog.initialize(t_mode::NORMAL, t_interrupt::OVF);
	_watchdog.setPrescaler(64);
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::update(void)
{
	//Timer.
	_watchdog.reset();
	
	//Update measurements.
	measure();
	
	//Update receiver inputs.
	receive();
	
	//Flying.
	enableMotors();
	if(_safety==0x02)driveMotors();
	
	//Non-flying.
	disableMotors();
	
	//Get calculations time.
	//Debugging purposes.
	_looptime = _watchdog.getNonResetCount();
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::measure(void)
{
	//Update accelero.
	_imu->setAccelero(&_rawAccel);
	_bodyAccel = _bodyAccel.multiply(0.9f).sum(_rawAccel.multiply(0.1f));
	
	//Update gyro.
	if (!_imu->getMovement(&_bodyAccel, 1.02))_imu->setBias(&_rawBias); 	//Update bias if not moving.
	_imu->setGyroscope(&_rawOmega, &_rawBias);
	_bodyOmega  = _rawOmega;
	
	//Update rotation quaternion.
	_imu->setQuaternion(&_qAtt, &_bodyOmega, &_bodyAccel);
	
	//Update world vectors.
	_worldAccel = (_qAtt.conj()).rotate(_bodyAccel);
	_worldAccel.z -= 0.91f;
	
	//Battery volage.
	//TODO::
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::receive(void)
{
	//Receiver channels.
	_dcDesiredThrottle = _rec->getThrottleChannel();
	_dcDesiredRoll     = _rec->getRollChannel();
	_dcDesiredPitch    = _rec->getPitchChannel();
	_dcDesiredYaw      = _rec->getYawChannel();
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::enableMotors(void)
{
	// Make sure the throttle stick is placed in the left-bottom position.
	if(_dcDesiredThrottle < 0.05 and _dcDesiredYaw < 0.05)
	{
		_safety = 0x01;
	}
	
	// Make sure the throttle stick is placed back in the centre-bottom position.
	if(_safety==0x01 and _dcDesiredThrottle < 0.05 and _dcDesiredYaw > 0.45 and _dcDesiredYaw < 0.55)
	{
		// Increase safety state.
		_safety = 0x02;
		
		// Start the motors at low speed.
		_esc1->writeSpeed(0.1);
		_esc2->writeSpeed(0.1);
		_esc3->writeSpeed(0.1);
		_esc4->writeSpeed(0.1);
	}
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::disableMotors(void)
{	
	// Make sure the throttle stick is placed in the left-bottom corner.
	if(_safety==0x02 and _dcDesiredThrottle < 0.05 and _dcDesiredYaw > 0.95)
	{
		_safety = 0x03;
	}
	
	// Make sure the throttle stick is placed back in the centre-bottom position.
	if(_safety==0x03 and _dcDesiredThrottle < 0.05 and _dcDesiredYaw > 0.45 and _dcDesiredYaw < 0.55)
	{
		// Reset safety state.
		_safety &= 0x00;
		
		// Stop the motors.
		_esc1->writeSpeed(0.0);
		_esc2->writeSpeed(0.0);
		_esc3->writeSpeed(0.0);
		_esc4->writeSpeed(0.0);
	}
}

/*******************************************************************************
 * 
 ******************************************************************************/
void controller::driveMotors(void)
{
	vector dcOutput;
	
	float dcEsc1;
	float dcEsc2;
	float dcEsc3;
	float dcEsc4;
	
	//PID feedback output in duty cycle.
	dcOutput = getFeedbackDc();
	
	//TODO::Map max throttle to 80% instead of 100%.
	if (_dcDesiredThrottle > 0.8)_dcDesiredThrottle=0.8;
	
	//Do calculations based on structure.
	//TODO::add more structures.
	switch(_layout)
	{
		case(c_layout::CROSS):
			dcEsc1 = _dcDesiredThrottle - dcOutput.x + dcOutput.y - dcOutput.z;
			dcEsc2 = _dcDesiredThrottle + dcOutput.x + dcOutput.y + dcOutput.z;
			dcEsc3 = _dcDesiredThrottle + dcOutput.x - dcOutput.y - dcOutput.z;
			dcEsc4 = _dcDesiredThrottle - dcOutput.x - dcOutput.y + dcOutput.z;
		
		case(c_layout::PLUS):
			dcEsc1 = _dcDesiredThrottle - dcOutput.x - dcOutput.z;
			dcEsc2 = _dcDesiredThrottle + dcOutput.y + dcOutput.z;
			dcEsc3 = _dcDesiredThrottle + dcOutput.x - dcOutput.z;
			dcEsc4 = _dcDesiredThrottle - dcOutput.y + dcOutput.z;
			
		case(c_layout::NONE):
		default:
			;;
	}
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
	_esc1->writeSpeed(dcEsc1);
	_esc2->writeSpeed(dcEsc2);
	_esc3->writeSpeed(dcEsc3);
	_esc4->writeSpeed(dcEsc4);
}

/*******************************************************************************
 * 
 ******************************************************************************/
vector controller::getFeedbackDc(void)
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
	// Introduce a little deadband to compensate for high sensivity.
	if(_dcDesiredRoll  < _dcDeadbandLowerBoundary or _dcDesiredRoll  > _dcDeadbandUpperBoundary)roll  = _dcDesiredRoll*_maxRoll;
	if(_dcDesiredPitch < _dcDeadbandLowerBoundary or _dcDesiredPitch > _dcDeadbandUpperBoundary)pitch = _dcDesiredPitch*_maxPitch;
	if(_dcDesiredYaw   < _dcDeadbandLowerBoundary or _dcDesiredYaw   > _dcDeadbandUpperBoundary)yaw   = _dcDesiredYaw*_maxYaw;
	
	// Consequtive quaternion rotations.
	qRoll  = quaterion(roll,  _unitY);
	qPitch = quaterion(pitch, _unitX);
	qYaw   = quaterion(yaw,   _unitZ);
	qDes   = qYaw.cross(qPitch.cross(qRoll));
	
	// Calculate PID rotational velocity outputs [rad/s].
	float dRoll  = _pidRoll->calculate(_qAtt.y,  qDes.y);
	float dPitch = _pidPitch->calculate(_qAtt.x, qDes.x);
	float dYaw   = _pidYaw->calculate(_qAtt.z,   qDes.z);
	
	// Transform back to duty cycle [%].
	float dcRoll  = dRoll/(DPS_MAX*pi/180);
	float dcPitch = dPitch/(DPS_MAX*pi/180);
	float dcYaw   = dYaw/(DPS_MAX*pi/180);
	
	return vector(dcPitch, dcRoll, dcYaw);
}

/*******************************************************************************
 * 
 ******************************************************************************/
float controller::getLooptime()
{
	return _looptime;
}

/*******************************************************************************
 * Method for returning the quadcopters state vector.
 * 
 * 	[g_Pitch   , g_Roll   , g_Yaw,
 * 	 g_dotPitch, g_dotRoll, g_dotYaw,
 * 	 g_accelX  , g_accelY , g_accelZ]^T
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
int8_t controller::monitorBattery()
{
	int8_t ret = 0;
	
	if(..bat..)ret = -1;
	
	return ret;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t controller::getSafetyState()
{
	return _safety;
}
