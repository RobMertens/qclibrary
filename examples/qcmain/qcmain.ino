/**
 * INCLUDES.
 * Include qclibrary and definitions.
 */
#include <definitions.h> //MODIFY THIS FILE!
#include <qclibrary.h>

/**
 * DECLARATIONS.
 * The quadcopter hardware instances are initiated here.
 * For now global vectors/quaternions are stored in the ino-file.
 */
//RX instance.
RX receiver = new RX(void);

//PID instances.
MPU6050 sensingDevice = new MPU6050(MPU_ADDRESS);

//PID instances.
PID pitchControl = new PID(PITCHPGAIN,
                           PITCHIGAIN,
                           PITCHDGAIN,
                           FORWARD,
                           PITCHMINLIMIT,
                           PITCHMAXLIMIT);                 
PID rollControl  = new PID(ROLLPGAIN,
                           ROLLIGAIN,
                           ROLLDGAIN,
                           FORWARD,
                           ROLLMINLIMIT,
                           ROLLMAXLIMIT);
PID yawControl   = new PID(YAWPGAIN,
                           YAWIGAIN,
                           YAWDGAIN,
                           FORWARD,
                           YAWMINLIMIT,
                           YAWMAXLIMIT);

//Motor instances.
ESC motors  = new ESC(MINPULSEWIDTH,
                      MAXPULSEWIDTH);

//LED instance.
LED indicator = new LED(void);

//Battery instance.
BAT battery = new BAT(void);

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

//Timers.
//t index stands for _t_imer
unsigned long watchdog;
uint16_t tLoop;
uint16_t tEsc;
uint16_t tMpu;

//Math.
const float us = 0.000001f;
const float pi = 3.141592635898f;
const float g  = 9.81f;

/**
 * MAIN LOOP.
 */
int main(void){
  for (;;)
  {
    //Initialize watchdog.
    watchdog = micros();
    
    //Update measurements
    sensingDevice.setAcceleration(&mAccel);
    bAccel = bAccel.multiply(0.9f).sum(mAccel.multiply(0.1f)); //Filter out noise w/ complementary filter.
    if (!getMovement(&bAccel, 1.0))sensingDevice.setBias(&bias); //Update bias if not moving.
    sensingDevice.setOmega(&mOmega, &mBias);
    
    //Calculate new state vectors.
    sensingDevice.setQuaternion(&rQ, &bOmega, &bAccel, tLoop); //TODO::Fix setQuaternion(...)
    wAccel = (rQ.getConjugate()).rotate(bAccel);wAccel.z -= 0.91f; // Remove gravity;
    wVeloc = wVeloc.sum(wAccel.multiply(g*us*tLoop)); //CRAP...
    
    while(tLoop <= 4000)
    {
      tLoop = micros() - watchdog;
    }
  }
}

/**
 * EXTERNAL FUNCTIONS.
 */
bool getMovement(vector *a, float p)
{
  //Local variables.
  bool movement;
  float eta;

  //If acc/g-ratio equals greater than p[%]: dynamic flight.
  //Else: take-off/landing position or hoovering.
  //Sensitivity depends on p, different for bias/quaternion.
  //GRAVITY VALUE DEPENDENT ON SENSOR.
  movement = 0;
  eta = abs((a -> m)/0.91f);
  if (eta >= p)movement = 1;

  //Return movement status.
  return movement;
}
