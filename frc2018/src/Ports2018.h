#ifndef SRC_PORTS2018_H_
#define SRC_PORTS2018_H_

/**
 * Ports
 */

/* ***************************** ROBOT PORTS **************************** */

static const bool IS_WRIST_MOTORIZED					= false;

/* ------------------- DRIVE TALON IDS ------------------ */
static const int LEFT_DRIVE_MASTER_ID					= 3;
static const int LEFT_DRIVE_SLAVE_ID 		 			= 4;
static const int RIGHT_DRIVE_MASTER_ID					= 1;
static const int RIGHT_DRIVE_SLAVE_ID 					= 2;

/* ---------------------- PWM PORTS --------------------- */

// Superstructure PWM Ports
static const int LEFT_INTAKE_MOTOR_PWM_PORT				= 9;
static const int RIGHT_INTAKE_MOTOR_PWM_PORT			= 8;
static const int ELEVATOR_MOTOR_PWM_PORT				= 7;
static const int RAMP_L_MOTOR_PWM_PORT					= 0; // TODO CHANGE THIS
static const int RAMP_R_MOTOR_PWM_PORT					= 0; // TODO CHANGE THIS
static const int WRIST_MOTOR_PWM_PORT					= 6;

/* --------------------- PDP CHANNELS ------------------- */
static const int ELEVATOR_MOTOR_PDP_CHAN				= 3;
static const int LEFT_DRIVE_MOTOR_A_PDP_CHAN            = 15;
static const int LEFT_DRIVE_MOTOR_B_PDP_CHAN            = 14;
static const int RIGHT_DRIVE_MOTOR_A_PDP_CHAN           = 0;
static const int RIGHT_DRIVE_MOTOR_B_PDP_CHAN           = 1;
static const int LEFT_INTAKE_MOTOR_PDP_CHAN				= 10;
static const int RIGHT_INTAKE_MOTOR_PDP_CHAN            = 5;
static const int WRIST_MOTOR_PDP_CHAN					= 13;

/* ------------------ DIGITAL I/O PORTS ----------------- */
static const int LEFT_DRIVE_ENCODER_YELLOW_PWM_PORT		= 0;
static const int LEFT_DRIVE_ENCODER_RED_PWM_PORT		= 1;

static const int RIGHT_DRIVE_ENCODER_YELLOW_PWM_PORT	= 2;
static const int RIGHT_DRIVE_ENCODER_RED_PWM_PORT		= 3;

static const int ELEVATOR_ENCODER_YELLOW_PWM_PORT		= 4;
static const int ELEVATOR_ENCODER_RED_PWM_PORT			= 5;
static const int ELEVATOR_BOTTOM_LIMIT_SWITCH_PORT      = 6;
static const int ELEVATOR_TOP_LIMIT_SWITCH_PORT         = 7;

static const int INTAKE_SENSOR_PWM_PORT					= 6; // TODO CHANGE THIS

/* ------------------ ANALOG IN PORTS --------------------*/

static const int PRESSURE_SENSOR_PORT					= 0; // TODO CHANGE THIS
static const int WRIST_POT_PORT							= 1; //	TODO CHANGE THIS

/* ------------------------ MISC -------------------------*/
static const int PNEUMATICS_CONTROL_MODULE_ID			= 0;

/* ------------------- SOLENOID PORTS ------------------- */
static const int GEAR_SHIFT_FORWARD_SOLENOID_PORT		= 0;
static const int GEAR_SHIFT_REVERSE_SOLENOID_PORT		= 1;

static const int WRIST_UP_SOLENOID_PORT					= 2;
static const int WRIST_DOWN_SOLENOID_PORT				= 3;

static const int RAMP_LEG_SOLENOID_L_PORT_A				= 0; // TODO CHANGE THIS
static const int RAMP_LEG_SOLENOID_L_PORT_B				= 0; // TODO CHANGE THIS
static const int RAMP_LEG_SOLENOID_R_PORT_A				= 0; // TODO CHANGE THIS
static const int RAMP_LEG_SOLENOID_R_PORT_B				= 0; // TODO CHANGE THIS
static const int RAMP_RELEASE_SOLENOID_L_PORT_A			= 0; // TODO CHANGE THIS
static const int RAMP_RELEASE_SOLENOID_L_PORT_B			= 0; // TODO CHANGE THIS
static const int RAMP_RELEASE_SOLENOID_R_PORT_A			= 0; // TODO CHANGE THIS
static const int RAMP_RELEASE_SOLENOID_R_PORT_B			= 0; // TODO CHANGE THIS

/* ************************ DRIVER STATION PORTS ************************ */

/* ----------------- JOYSTICK USB PORTS ----------------- */
static const int LEFT_JOY_USB_PORT						= 0;
static const int RIGHT_JOY_USB_PORT						= 1;
static const int OPERATOR_JOY_USB_PORT					= 2;
static const int OPERATOR_JOY_B_USB_PORT				= 3;

/* -------------------- BUTTON PORTS -------------------- */

// Drive controller button ports
static const int DRIVER_OUTTAKE_BUTTON_PORT                    = 2; //left joystick
static const int DRIVE_DIRECTION_BUTTON_PORT			= 3;
static const int HIGH_LOW_GEAR_BUTTON_PORT				= 3;
static const int ARCADE_DRIVE_BUTTON_PORT				= 6;
static const int QUICK_TURN_BUTTON_PORT					= 1;
static const int ALIGN_WITH_CUBE_BUTTON_PORT			= 2;

// Superstructure controller button ports
static const int STOP_TOP_LIMITSWITCH_BUTTON_PORT       = 7;  //WRONG: find a switch dudes
static const int STOP_BOTTOM_LIMITSWITCH_BUTTON_PORT    = 7;  //WRONG: find a switch dudes
static const int INTAKE_BUTTON_PORT						= 2;
static const int OUTTAKE_BUTTON_PORT					= 1;
static const int OUTTAKE_FAST_BUTTON_PORT				= 3;	// TODO CHECK THIS
static const int INTAKE_HOLD_SWITCH_PORT				= 7; //unused
static const int ELEVATOR_UP_BUTTON_PORT				= 8;
static const int ELEVATOR_DOWN_BUTTON_PORT				= 7; //unused
static const int RAMP_RELEASE_BUTTON_PORT				= 6;
static const int RAMP_RAISE_L_BUTTON_PORT				= 2;
static const int RAMP_RAISE_R_BUTTON_PORT				= 8;
static const int WRIST_BUTTON_PORT						= 9; //2;
static const int WRIST_UP_BUTTON_PORT					= 4;	// TODO label this
static const int WRIST_DOWN_BUTTON_PORT					= 6;
static const int WRIST_STALL_BUTTON_PORT				= 5;

// Auto mode switch ports
static const int LEFT_AUTO_SWITCH_PORT					= 2;
static const int MIDDLE_AUTO_SWITCH_PORT				= 3;
static const int RIGHT_AUTO_SWITCH_PORT					= 4;

#endif /* SRC_PORTS2018_H_ */
