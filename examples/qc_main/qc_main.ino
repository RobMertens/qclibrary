/**
 * INCLUDES.
 * Include qclibrary and definitions.
 */
#include <qcdef.h> //MODIFY THIS FILE!
#include <qclibrary.h>

/**
 * DECLARATIONS.
 * The quadcopter hardware instances are initiated here.
 * For now global vectors/quaternions are stored in the ino-file.
 */
//RX instance.
RX receiver(&RX_PIN, &RX_PCMSK, RX_PCINT, RX_PCIE);

//PID instances.
MPU6050 sensor(S_ADDRESS);

//PID instances.
PID pitchControl(P_PITCH_KP, P_PITCH_KI, P_PITCH_KD, P_PITCH_MAX, P_PITCH_MIN);
PID rollControl(P_ROLL_KP, P_ROLL_KI, P_ROLL_KD, P_ROLL_MAX, P_ROLL_MIN);
PID yawControl(P_YAW_KP, P_YAW_KI, P_YAW_KD, P_YAW_MAX, P_YAW_MIN);

//Motor instances.
ESC motors(&E_DDR, E_DDRMSK);

//LED instance.
LED led(&L_DDR, L_DDRMSK, &L_PIN, &L_PORT);

//Battery instance.
BAT battery(void);

//Raw vectors from MPU6050 measurement.
//m index stands for _m_easured
vector mOmega;
vector mAccel;
vector mBias;

//Body vectors w/ removed bias and noise.
//b index stands for _b_ody
vector bOmega;
vector bAccel;

//Rotation quaternions.
//r index stands for _r_otation
quaternion rQ;
vector rEuler; //RPY angles.
vector rTheta; //True angles w.r.t. Euler by integrating factors.

//World vectors.
//These vectors contain the most usefull information about our quadcopter.
//The YAW angle measurement cannot be corrected because I don't have a magnetometer.
//This leads to poor YAW control.
//w index stands for _w_orld
vector wOmega;
vector wPosit;
vector wVeloc; //After integration. (crap)
vector wAccel; //Gravity compensated.

/**
 * MAIN LOOP.
 */
int main(void){
	for (;;)
	{

		//Update measurements
		sensingDevice.setAcceleration(&mAccel);
		bAccel = bAccel.multiply(0.9f).sum(mAccel.multiply(0.1f)); //Filter out noise w/ complementary filter.
		if (!getMovement(&bAccel, 1.0))sensingDevice.setBias(&bias); //Update bias if not moving.
		sensingDevice.setOmega(&mOmega, &mBias);

		//Calculate new state vectors.
		sensor.setQuaternion(&rQ, &bOmega, &bAccel, tLoop); //TODO::Fix setQuaternion(...)
		wAccel = (rQ.getConjugate()).rotate(bAccel);wAccel.z -= 0.91f; // Remove gravity;
		wVeloc = wVeloc.sum(wAccel.multiply(g*us*tLoop)); //CRAP...
	}
}

/**
 * EXTERNAL FUNCTIONS.
 * Should be empty.
 */
