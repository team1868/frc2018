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
	alignWithCubeDesired_ = false;

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

	intakeButton_ = new ButtonReader(operatorJoyB_, INTAKE_BUTTON_PORT);
	outtakeFastButton_ = new ButtonReader(operatorJoyB_, OUTTAKE_FAST_BUTTON_PORT);
	outtakeButton_ = new ButtonReader(operatorJoyB_, OUTTAKE_BUTTON_PORT);
	intakeHoldSwitch_ = new ButtonReader(operatorJoy_, INTAKE_HOLD_SWITCH_PORT);
	elevatorUpButton_ = new ButtonReader(operatorJoyB_, ELEVATOR_UP_BUTTON_PORT);
	elevatorDownButton_ = new ButtonReader(operatorJoyB_, ELEVATOR_DOWN_BUTTON_PORT);
	rampReleaseButton_ = new ButtonReader(operatorJoyB_, RAMP_RELEASE_BUTTON_PORT);
	rampRaiseLButton_ = new ButtonReader(operatorJoy_, RAMP_RAISE_L_BUTTON_PORT);
	rampRaiseRButton_ = new ButtonReader(operatorJoy_, RAMP_RAISE_R_BUTTON_PORT);
	wristSwitch_ = new ButtonReader(operatorJoy_, WRIST_BUTTON_PORT);
	alignWithCubeButton_ = new ButtonReader(rightJoy_, ALIGN_WITH_CUBE_BUTTON_PORT);

	intakeDesired_ = false;
	outtakeDesired_ = false;
	outtakeFastDesired_ = false;
	intakeHoldDesired_ = false;
	elevatorUpDesired_ = false;
	elevatorDownDesired_ = false;
	rampReleaseDesired_ = false;
	wristUpDesired_ = false;
	wristDownDesired_ = false;

	leftAutoSwitch_ = new ButtonReader(operatorJoy_,LEFT_AUTO_SWITCH_PORT);
	rightAutoSwitch_ = new ButtonReader(operatorJoy_, RIGHT_AUTO_SWITCH_PORT);
	middleAutoSwitch_ = new ButtonReader(operatorJoy_, MIDDLE_AUTO_SWITCH_PORT);

	ReadControls();
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
	alignWithCubeDesired_ = alignWithCubeButton_->WasJustPressed();

	intakeDesired_ = intakeButton_->IsDown();
	outtakeDesired_ = outtakeButton_->IsDown();
	outtakeFastDesired_ = outtakeFastButton_->IsDown();
	intakeHoldDesired_ = intakeHoldSwitch_->IsDown();
	elevatorUpDesired_ = elevatorUpButton_->IsDown();
	elevatorDownDesired_ = elevatorDownButton_->IsDown();

	rampReleaseDesired_ = rampReleaseButton_->WasJustPressed();
	if (wristSwitch_->WasJustPressed()) {
		wristUpDesired_ = true;
	} else {
		wristUpDesired_ = false;
	}

	if (wristSwitch_->WasJustReleased()) {
		wristDownDesired_ = true;
	} else {
		wristDownDesired_ = false;
	}

	// Reading Auto Switch vals;
	leftDown_ = leftAutoSwitch_->IsDown();
	middleDown_ = middleAutoSwitch_->IsDown();
	rightDown_ = rightAutoSwitch_->IsDown();
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

bool ControlBoard::GetAlignWithCubeDesired() {
	return alignWithCubeDesired_;
}

bool ControlBoard::GetIntakeDesired() {
	return intakeDesired_;
}

void ControlBoard::SetIntakeDesired(bool desired) {
	intakeDesired_ = desired;
}


bool ControlBoard::GetOuttakeDesired() {
	return outtakeDesired_;
}

bool ControlBoard::GetOuttakeFastDesired() {
	return outtakeFastDesired_;
}

bool ControlBoard::GetIntakeHoldDesired() {
	return intakeHoldDesired_;
}

bool ControlBoard::GetElevatorUpDesired() {
	return elevatorUpDesired_;
}

bool ControlBoard::GetElevatorDownDesired() {
	return elevatorDownDesired_;
}

bool ControlBoard::GetRampReleaseDesired() {
	return rampReleaseDesired_;
}

bool ControlBoard::GetRampRaiseLDesired() {
	return rampRaiseLDesired_;
}

bool ControlBoard::GetRampRaiseRDesired() {
	return rampRaiseRDesired_;
}

bool ControlBoard::GetWristUpDesired() {
	return wristUpDesired_;
}

bool ControlBoard::GetWristDownDesired() {
	return wristDownDesired_;
}

double ControlBoard::GetElevatorDialOutput() {
	return 0.0;
}

AutoMode::AutoPositions ControlBoard::GetDesiredAutoPosition() {
	AutoMode::AutoPositions position = AutoMode::kBlank;

	if (!leftDown_) {
		if (!middleDown_) {
			if (!rightDown_){
				position = AutoMode::kMiddle;	// starting position middle
			} else {
				position = AutoMode::kFarRight;	// starting position far right
			}
		} else {
			if (rightDown_) {
				position = AutoMode::kMiddleRight;	// starting position middle right
			}
		}
	} else {
		if (!middleDown_) {
			if(!rightDown_) {
				position = AutoMode::kLeft;	// starting position left
			}
		} else {
			if(rightDown_) {
				position = AutoMode::kIni;
			}
		}
	}

	return position;
}

void ControlBoard::ReadAllButtons() {
	driveDirectionButton_->ReadValue();
	gearShiftButton_->ReadValue();
	arcadeDriveButton_->ReadValue();
	quickTurnButton_->ReadValue();
	alignWithCubeButton_->ReadValue();

	intakeButton_->ReadValue();
	outtakeButton_->ReadValue();
	outtakeFastButton_->ReadValue();
	intakeHoldSwitch_->ReadValue();
	elevatorUpButton_->ReadValue();
	elevatorDownButton_->ReadValue();
	rampReleaseButton_->ReadValue();
	rampRaiseLButton_->ReadValue();
	rampRaiseRButton_->ReadValue();
	wristSwitch_->ReadValue();

	rightAutoSwitch_->ReadValue();
	middleAutoSwitch_->ReadValue();
	leftAutoSwitch_->ReadValue();
}

ControlBoard::~ControlBoard() {
	// TODO Auto-generated destructor stub
}

