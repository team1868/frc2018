#ifndef SRC_DRIVERSTATION_CONTROLBOARD_H_
#define SRC_DRIVERSTATION_CONTROLBOARD_H_

#include <WPILib.h>
#include <DriverStation/ButtonReader.h>
#include <Ports2018.h>

class ControlBoard {
public:
	enum Joysticks{ kLeftJoy, kRightJoy };
	enum Axes{ kX, kY, kZ };

	/**
	 * ControlBoard constructor initializes joysticks, buttons, and all other values
	 */
	ControlBoard();

	/**
	 * Reads all control values for buttons and joysticks
	 */
	void ReadControls();

	/**
	 * Returns joystick value according to specified joystick and axis
	 */
	double GetJoystickValue(Joysticks j, Axes a);
	bool GetReverseDriveDesired();
	bool GetHighGearDesired();
	bool GetArcadeDriveDesired();
	bool GetQuickTurnDesired();

	/**
	 * Returns desired values for superstructure controls
	 */
	bool GetIntakeDesired();
	void SetIntakeDesired(bool desired);
	bool GetOuttakeDesired();
	bool GetElevatorUpDesired();
	bool GetElevatorDownDesired();
	bool GetElevatorHeightDesired();
	double GetElevatorHeightValue();

	virtual ~ControlBoard();
private:
	void ReadAllButtons();

	// Joystick values
	double leftJoyX_, leftJoyY_, leftJoyZ_, rightJoyX_, rightJoyY_, rightJoyZ_;

	// Drive Modes
	bool reverseDriveDesired_, highGearDesired_, arcadeDriveDesired_, quickTurnDesired_;

	// Joysticks for drive
	Joystick *leftJoy_, *rightJoy_;

	// Joysticks for operator
	Joystick *operatorJoy_, *operatorJoyB_;

	// Buttons for drive
	ButtonReader *driveDirectionButton_, *gearShiftButton_, *arcadeDriveButton_, *quickTurnButton_;

	// Auto switches
	ButtonReader *leftAutoSwitch_, *rightAutoSwitch_, *middleAutoSwitch_;

	// Buttons for superstructure
	ButtonReader *intakeButton_, *outtakeButton_, *elevatorUpButton_, *elevatorDownButton_, *elevatorHeightButton_;

	// Variables for superstructure
	bool intakeDesired_, outtakeDesired_, elevatorUpDesired_, elevatorDownDesired_, elevatorHeightDesired_;
	double elevatorHeightValue_;
};

#endif /* SRC_DRIVERSTATION_CONTROLBOARD_H_ */
