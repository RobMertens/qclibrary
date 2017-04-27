#ifndef _QCDEF_H_
#define _QCDEF_H_

#include <avr/io.h>

/*******************************************************************************
 * Quadcopter-Library-v1
 * QCDEF.H
 *  
 * This file contains predefined constants that mostly depend on hardware.
 * The definitions are filled in for my hardware setup which exist out of:
 * (1) 1x Arduino MEGA ADK (same pinout as MEGA)
 * (2) 1x Reely HT-5 TX/RX
 * (3) 1x MPU6050 w/ level shifter
 * (4) 4x Turnigy Plush 10A
 * (5) 4x DYS BE1806-2300kV
 *
 * If you are using another arduino make sure it works with 16MHz clock speed.
 * For addressing the  correct registers see your arduino type PINOUT-
 * diagram.
 *
 * @author	Rob Mertens
 * @date	14/08/2016
 * @version	1.1.1
 ******************************************************************************/

/* REGISTER SETTINGS AND VALUES*/

// LED: index L
#define L_DDR				DDRB		// Data Direction Register for the LED.
#define L_DDRMSK			0x80		// Data Direction Register value (0b10000000).
							// This is arduino pin 13 (PB7).
#define L_PIN				PINB		// Data Input Register for the LED.
#define L_PORT	 			PORTB		// Data Output Register for the LED.

// ESCs: index E
#define E_DDR	  			DDRC		// Data Direction Register for the ESC's.
#define E_DDRMSK			0x0F		// Data Direction Register value (0b00001111).
							// These are arduino pins 34, 35, 36 and 37 (PC0, PC1, PC2 and PC3).
#define E_PIN				PINC		// Data Input Register for the ESC's.
#define E_PORT	 			PORTC		// Data Output Register for the ESC's.
#define E_MINPULSEWIDTH			700		// Minimum microsecond value [µs].
#define E_MAXPULSEWIDTH			2000		// Maximum microsecond value [µs].

// BATTERY: index B
#define B_ALARMLEVEL 			1110		// Battery low voltage boundary (ex.: 1110 equals 11.10V).

// PID: index P
#define P_PITCH_KP			1.0
#define P_PITCH_KI			1.0
#define P_PITCH_KD			1.0
#define P_PITCH_MAX			100.0
#define P_PITCH_MIN			100.0

#define P_ROLL_KP			1.0
#define P_ROLL_KI			1.0
#define P_ROLL_KD			1.0
#define P_ROLL_MAX			100.0
#define P_ROLL_MIN			100.0

#define P_YAW_KP			1.0
#define P_YAW_KI			1.0
#define P_YAW_KD			1.0
#define P_YAW_MAX			100.0
#define P_YAW_MIN			100.0

#define FORWARD				1
#define REVERSE				-1

// MPU6050: index M
#define MPU_ADDRESS 			0x68
#define MPU_SCALE	 		0x08
               
// RX: index R
#define RX_PIN				PINB		// Pin Input Register.
#define RX_PCINT			0x0F		// Pin Change Interrupt pins (0b00001111).
							// These are 53, 52, 51 and 50 (PCINT0, PCINT1, PCINT2 and PCINT3).
#define RX_PCIE				0x01		// Pin Change Interrupt Enable.
							// For PCINT0-7 	0x01
							// For PCINT8-15	0x02 (MEGA only)
							// For PCINT16-23	0x04 (MEGA only)
#define RX_PCMSK			PCMSK0		// Pin Change Mask Register.
							// For PCINT0-7 	PCMSK0
							// For PCINT8-15	PCMSK1 (MEGA only)
							// For PCINT16-23	PCMSK2 (MEGA only)


#endif





