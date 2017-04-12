#include <qclibrary.h>

timer t(&TCCR1A, &TCNT1, &TIMSK1);

// INIT
void setup()
{
	Serial.begin(9600);
	
	t.initialize(0x00, 0x01);
}

// MAIN PROGRAM
void loop()
{
	Serial.println(t.getOverflow());
}



