#include <DriverStation/ControlBoard.h>

#include "WPILib.h"

ControlBoard::ControlBoard() {
	leftJoyX_ = 0.0;
	leftJoyY_ = 0.0;
	leftJoyZ_ = 0.0;
	rightJoyX_ = 0.0;
	rightJoyY_ = 0.0;
	rightJoyZ_ = 0.0;

	reverseDriveDesired_ = false;
	highGearDesired_ = true; // TODO may want to fix this
	arcadeDriveDesired_ = true;
	quickTurnDesired_ = true;

	leftJoy_ = new Joystick(LEFT_JOY_USB_PORT);
	rightJoy_ = new Joystick(RIGHT_JOY_USB_PORT);
	operatorJoy_ = new Joystick(OPERATOR_JOY_USB_PORT);
	operatorJoyB_ = new Joystick(OPERATOR_JOY_B_USB_PORT);

	gearShiftButton_ = new ButtonReader(rightJoy_, HIGH_LOW_GEAR_BUTTON_PORT);
	arcadeDriveButton_ = new ButtonReader(operatorJoy_, ARCADE_DRIVE_BUTTON_PORT); // TODO change this
	quickTurnButton_ = new ButtonReader(rightJoy_, QUICK_TURN_BUTTON_PORT);
	driveDirectionButton_ = new ButtonReader(leftJoy_, DRIVE_DIRECTION_BUTTON_PORT);

	leftAutoSwitch_ = new ButtonReader(operatorJoyB_,LEFT_AUTO_SWITCH_PORT);
	rightAutoSwitch_ = new ButtonReader(operatorJoyB_, RIGHT_AUTO_SWITCH_PORT);
	middleAutoSwitch_ = new ButtonReader(operatorJoyB_, MIDDLE_AUTO_SWITCH_PORT);
}

void ControlBoard::ReadControls() {
	ReadAllButtons();

	//Reading joystick values
	leftJoyX_ = leftJoy_->GetX();
	leftJoyY_ = leftJoy_->GetY();
	leftJoyZ_ = leftJoy_->GetZ();
	rightJoyX_ = rightJoy_->GetX();
	rightJoyY_ = rightJoy_->GetY();
	rightJoyZ_ = rightJoy_->GetZ();

	reverseDriveDesired_ = driveDirectionButton_->IsDown();
	highGearDesired_ = gearShiftButton_->IsDown();
	arcadeDriveDesired_ = arcadeDriveButton_->IsDown();
	quickTurnDesired_ = quickTurnButton_->IsDown();
}

double ControlBoard::GetJoystickValue(Joysticks j, Axes a) {
	switch (j) {
	case (kLeftJoy):
			switch(a) {
			case(kX):
					return leftJoyX_;
			case(kY):
					return leftJoyY_;
			case(kZ):
					return leftJoyZ_;
			}
	break;
	case (kRightJoy):
			switch(a){
			case(kX):
					return rightJoyX_;
			case(kY):
					return rightJoyY_;
			case(kZ):
					return rightJoyZ_;
			}
	break;
	}
	return 0;
}

bool ControlBoard::GetReverseDriveDesired() {
	return reverseDriveDesired_;
}

bool ControlBoard::GetHighGearDesired() {
	return highGearDesired_;
}

bool ControlBoard::GetArcadeDriveDesired() {
	return arcadeDriveDesired_;
}

bool ControlBoard::GetQuickTurnDesired() {
	return quickTurnDesired_;
}

void ControlBoard::ReadAllButtons() {
	driveDirectionButton_->ReadValue();
	gearShiftButton_->ReadValue();
	arcadeDriveButton_->ReadValue();
	quickTurnButton_->ReadValue();
}

ControlBoard::~ControlBoard() {
	// TODO Auto-generated destructor stub
}

