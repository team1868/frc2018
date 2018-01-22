#ifndef SRC_PORTS2018_H_
#define SRC_PORTS2018_H_

/**
 * Ports
 */

/* ***************************** ROBOT PORTS **************************** */

/* ------------------- DRIVE TALON IDS ------------------ */
static const int LEFT_DRIVE_MASTER_ID					= 3;
static const int LEFT_DRIVE_SLAVE_ID 		 			= 2;
static const int RIGHT_DRIVE_MASTER_ID					= 4;
static const int RIGHT_DRIVE_SLAVE_ID 					= 1;

/* ---------------------- PWM PORTS --------------------- */


/* --------------------- PDP CHANNELS ------------------- */

/* ------------------ DIGITAL I/O PORTS ----------------- */
static const int LEFT_DRIVE_ENCODER_A_PWM_PORT			= 8;
static const int LEFT_DRIVE_ENCODER_B_PWM_PORT			= 9;

static const int RIGHT_DRIVE_ENCODER_A_PWM_PORT			= 5;
static const int RIGHT_DRIVE_ENCODER_B_PWM_PORT			= 6;

/* ------------------ ANALOG IN PORTS --------------------*/

/* ------------------------ MISC -------------------------*/
static const int PNEUMATICS_CONTROL_MODULE_ID			= 0;

/* ------------------- SOLENOID PORTS ------------------- */
static const int GEAR_SHIFT_FORWARD_SOLENOID_PORT		= 0;
static const int GEAR_SHIFT_REVERSE_SOLENOID_PORT		= 1;

/* ************************ DRIVER STATION PORTS ************************ */

/* ----------------- JOYSTICK USB PORTS ----------------- */
static const int LEFT_JOY_USB_PORT						= 0;
static const int RIGHT_JOY_USB_PORT						= 1;
static const int OPERATOR_JOY_USB_PORT					= 2;
static const int OPERATOR_JOY_B_USB_PORT				= 3;

/* -------------------- BUTTON PORTS -------------------- */

// Drive controller button ports
static const int DRIVE_DIRECTION_BUTTON_PORT			= 3;
static const int HIGH_LOW_GEAR_BUTTON_PORT				= 3;
static const int ARCADE_DRIVE_BUTTON_PORT				= 6;
static const int QUICK_TURN_BUTTON_PORT					= 2;

// Superstructure controller button ports

// Auto mode switch ports
static const int LEFT_AUTO_SWITCH_PORT					= 1;	// TODO CHECK THIS
static const int RIGHT_AUTO_SWITCH_PORT					= 2;	// TODO CHECK THIS
static const int MIDDLE_AUTO_SWITCH_PORT				= 3;	// TODO CHECK THIS

#endif /* SRC_PORTS2018_H_ */
