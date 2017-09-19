#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#include <avr/io.h>

/*******************************************************************************
 * Quadcopter-Library-v1
 * definitions.h
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
 * @date		14/08/2016
 * @version	1.1.1
 ******************************************************************************/

/* REGISTER SETTINGS AND VALUES*/

/*******************************************************************************
 * LED : index L
 ******************************************************************************/
#define L_DDR							DDRB		// Data Direction Register for the LED.
#define L_DDRMSK					0x80		// Data Direction Register value (0b10000000).
																	// This is arduino pin 13 (PB7).
#define L_PIN							PINB		// Data Input Register for the LED.
#define L_PORT	 					PORTB		// Data Output Register for the LED.

/*******************************************************************************
 * ESCs : index E
 * This describes the ESCs in general, port configuration and so forth.
 *
 * ESC1 : index E1
 * ESC2 : index E2
 * ESC3 : index E3
 * ESC4 : index E4
 * These indices point to values belonging to a specific ESC.
 ******************************************************************************/
#define E_DDR	  					DDRC		// Data Direction Register for the ESC's.
#define E_DDRMSK					0x0F		// Data Direction Register value (0b00001111).
																	// These are arduino pins 34, 35, 36 and 37 (PC0, PC1, PC2 and PC3).
#define E_PIN							PINC		// Data Input Register for the ESC's.
#define E_PORT	 					PORTC		// Data Output Register for the ESC's.
#define E_PERIOD					4000		// Maximum microsecond value [µs].
#define E_MAX							2000		// Maximum microsecond value [µs].
#define E_MIN							700			// Minimum microsecond value [µs].

//TIMER1 & TIMER3.
#define E_T_MODE					t_settings::mode::PWM_F
#define E_T_CHANNEL				t_settings::channel::BC_TOP
#define E_T_INVERTED			t_settings::inverted::NORMAL
#define E_T_PRESCALER			1
#define E_T_TOP						0xF9FF
#define E1_T_ALIAS				t_settings::alias::T1
#define E1_T_CHANNEL			t_settings::channel::B
#define E2_T_ALIAS				t_settings::alias::T3
#define E2_T_CHANNEL			t_settings::channel::B
#define E3_T_ALIAS				t_settings::alias::T1
#define E3_T_CHANNEL			t_settings::channel::C
#define E4_T_ALIAS				t_settings::alias::T3
#define E4_T_CHANNEL			t_settings::channel::C

/*******************************************************************************
 * BATTERY : index B
 ******************************************************************************/
#define B_ALARMLEVEL 			1110		// Battery low voltage boundary (ex.: 1110 equals 11.10V).

/*******************************************************************************
 * PID : index P
 ******************************************************************************/
#define P_P_KP						1.0		// Pitch
#define P_P_KI						1.0
#define P_P_KD						1.0
#define P_P_MAX						100.0
#define P_P_MIN					 -100.0

#define P_R_KP						1.0		// Roll
#define P_R_KI						1.0
#define P_R_KD						1.0
#define P_R_MAX						100.0
#define P_R_MIN					 -100.0

#define P_Y_KP						1.0		// Yaw
#define P_Y_KI						1.0
#define P_Y_KD						1.0
#define P_Y_MAX						100.0
#define P_Y_MIN					 -100.0

/*******************************************************************************
 * MPU6050: index IMU
 ******************************************************************************/
#define IMU_ADDRESS 			0x68
#define IMU_DPS_SCALE	 		0x08
#define IMU_ACC_SCALE	 		0x00

/*******************************************************************************
 * RX : index RX
 ******************************************************************************/
#define RX_PCINT					0x0F			// Pin Change Interrupt pins (0b00001111).
																		// These are 53, 52, 51 and 50 (PCINT0, PCINT1, PCINT2 and PCINT3).
#define RX_PCMSK					PCMSK0		// Pin Change Mask Register.
#define RX_PERIOD					4000
#define RX_MAX						2000
#define RX_MIN						1000
#define RX_MODE						rx_settings::mode::M1

//TIMER2.
#define RX_T_ALIAS				t_settings::alias::T2
#define RX_T_MODE					t_settings::mode::NORMAL
#define RX_T_INTERRUPT		t_settings::interrupt::OVF
#define RX_T_PRESCALER		1

/*******************************************************************************
 * Controller: index C
 ******************************************************************************/
#define C_DEADBAND				0.04

/*******************************************************************************
 * Watchdog : index W
 ******************************************************************************/
//TIMER2.
#define W_T_ALIAS					t_settings::alias::T2
#define W_T_MODE					t_settings::mode::NORMAL
#define W_T_INTERRUPT			t_settings::interrupt::OVF
#define W_T_PRESCALER 		64

/*******************************************************************************
 * Arbitrary globals, e.g. math values.
 ******************************************************************************/
//Numbers.
#define US 								0.000001f
#define PI 								3.141592635898f
#define GFORCE						9.81f
#define DEG2RAD						PI/180.0f
#define RAD2DEG						180.0f/PI

//Vectors and quaternions.
#define V_UNIT_X					vector::cptr(new vector(1.0f, 0.0f, 0.0f));
#define V_UNIT_X					vector::cptr(new vector(1.0f, 0.0f, 0.0f));
#define V_UNIT_X					vector::cptr(new vector(1.0f, 0.0f, 0.0f));
#define Q_REAL						quaternion::cptr(new quaternion());

//deprecated -> TODO
const vector::cptr controller::_MAXANGLE(new vector(0.785f, 0.785f, 6.283f));
_layout = layout;
c_settings::layout::CROSS, t_settings::alias::T2

#endif
