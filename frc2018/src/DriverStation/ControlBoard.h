#ifndef SRC_DRIVERSTATION_CONTROLBOARD_H_
#define SRC_DRIVERSTATION_CONTROLBOARD_H_

#include <WPILib.h>
#include <DriverStation/ButtonReader.h>
#include <Auto/Modes/AutoMode.h>
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
	bool GetAlignWithCubeDesired();
	bool GetDriverOuttakeDesired();

	/**
	 * Returns desired values for superstructure controls
	 */
	bool GetIntakeDesired();
	void SetIntakeDesired(bool desired);
	bool GetOuttakeDesired();
	bool GetOuttakeFastDesired();
	bool GetIntakeHoldDesired();
	bool GetHoldCubeDesired();
	bool GetElevatorUpDesired();
	bool GetElevatorDownDesired();
	bool GetTopLimitSwitchOffDesired();
	bool GetBottomLimitSwitchOffDesired();
	bool GetRampReleaseDesired();
	bool GetRampRaiseLDesired();
	bool GetRampRaiseRDesired();
	bool GetWristUpDesired();
	bool GetWristDownDesired();

	double GetElevatorDialOutput();

	// Auto Controls
	AutoMode::AutoPositions GetDesiredAutoPosition();

	virtual ~ControlBoard();
private:
	void ReadAllButtons();

	// Joystick values
	double leftJoyX_, leftJoyY_, leftJoyZ_, rightJoyX_, rightJoyY_, rightJoyZ_;

	// Drive Modes
	bool reverseDriveDesired_, highGearDesired_, arcadeDriveDesired_, quickTurnDesired_, alignWithCubeDesired_, driverOuttakeDesired_;

	// Joysticks for drive
	Joystick *leftJoy_, *rightJoy_;

	// Joysticks for operator
	Joystick *operatorJoy_, *operatorJoyB_;

	// Buttons for drive
	ButtonReader *driveDirectionButton_, *gearShiftButton_, *arcadeDriveButton_, *quickTurnButton_;

	// Buttons for superstructure
	ButtonReader *intakeButton_, *outtakeFastButton_, *outtakeButton_, *intakeHoldSwitch_, *elevatorUpButton_, *elevatorDownButton_,
				 *rampReleaseButton_, *rampRaiseLButton_, *rampRaiseRButton_, *wristSwitch_, *alignWithCubeButton_, *driverOuttakeButton_, *wristUpButton_,
				 *wristDownButton_, *wristStallButton_, *bottomLimitSwitchStopper_, *topLimitSwitchStopper_;

	// Variables for superstructure
	bool intakeDesired_, outtakeDesired_, outtakeFastDesired_, intakeHoldDesired_, elevatorUpDesired_, elevatorDownDesired_,
		 rampReleaseDesired_, rampRaiseLDesired_, rampRaiseRDesired_, wristUpDesired_, wristDownDesired_;

	// Auto switches
	bool leftDown_, rightDown_, middleDown_;
	ButtonReader *leftAutoSwitch_, *rightAutoSwitch_, *middleAutoSwitch_;
};

#endif /* SRC_DRIVERSTATION_CONTROLBOARD_H_ */
