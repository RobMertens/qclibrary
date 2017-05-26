/******************************************************************************
 * Quadcopter-Library-v1
 * qc_esc_receiver.ino
 * 
 * This is a test file for the receiver.
 *
 * @author:	Rob Mertens
 * @date:	10/05/2017
 * @version:	1.1.1
 ******************************************************************************/

//#include <definitions.h>
#include "RX.h"

/******************************************************************************
 * DECLARATIONS
 ******************************************************************************/
RX rec(&PCMSK0, 0x0F);

/******************************************************************************
 * MAIN
 ******************************************************************************/
int main(void)
{
	//Initialize.
	Serial.begin(9600);
	
	rec.initialize(rx_mode::M1);
	
	//Run motors.
	for(;;)
	{
		Serial.print("T: ");
		Serial.print(rec.getThrottleChannel());
		Serial.print("\t");
		
		Serial.print("R: ");
		Serial.print(rec.getRollChannel());
		Serial.print("\t");
		
		Serial.print("P: ");
		Serial.print(rec.getPitchChannel());
		Serial.print("\t");
		
		Serial.print("Y: ");
		Serial.print(rec.getYawChannel());
		Serial.print("\n");
	}
}
