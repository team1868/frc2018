/*
 * IntakeCommand.cpp
 *
 *  Created on: Jan 30, 2018
 *      Author: alisha
 */

#include <Auto/Commands/IntakeCommand.h>

IntakeCommand::IntakeCommand(RobotModel *robot) {

	isDone_ = false;
	robot_ = robot;
	intakeMotorOutput_ = 0.7; //TODO TEST

}

void IntakeCommand::Init() {
	isDone_ = false;
}

void IntakeCommand::Reset() {
	isDone_ = true;

}

void IntakeCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (robot_->GetCubeInIntake()){
		robot_->SetIntakeOutput(0.0);
		isDone_ = true;
	} else {
		robot_->SetIntakeOutput(intakeMotorOutput_);
	}

}

bool IntakeCommand::IsDone() {
	return isDone_;
}

IntakeCommand::~IntakeCommand() {
	// TODO Auto-generated destructor stub
	Reset();
}

