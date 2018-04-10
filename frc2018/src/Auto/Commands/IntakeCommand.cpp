/*
 * IntakeCommand.cpp
 *
 *  Created on: Jan 30, 2018
 *      Author: alisha
 */

#include <Auto/Commands/IntakeCommand.h>

IntakeCommand::IntakeCommand(RobotModel *robot, double intakeMotorOutput) : AutoCommand(){
	printf("Creating intake command\n");
	isDone_ = false;
	robot_ = robot;
	intakeMotorOutput_ = intakeMotorOutput;

	startTime_ = robot_->GetTime();
	timeDiff_ = robot_->GetTime() - startTime_;
	wasJustRunning_ = true;
}

void IntakeCommand::Init() {
	startTime_ = robot_->GetTime();
	timeDiff_ = robot_->GetTime() - startTime_;
	wasJustRunning_ = true;

	isDone_ = false;
}

void IntakeCommand::Reset() {
	isDone_ = true;

}

void IntakeCommand::Update(double currTimeSec, double deltaTimeSec) {
	timeDiff_ = robot_->GetTime() - startTime_;
	if (robot_->GetCubeInIntake()){
		robot_->SetIntakeOutput(0.0);
		isDone_ = true;
		printf("Intake Done!\n");
	} else {
		if (timeDiff_ <= 1.0) {
			robot_->SetIntakeOutput(intakeMotorOutput_);
		} else {
			if (wasJustRunning_ && timeDiff_ <= 1.2) {
				robot_->SetIntakeOutput(0.0);
				wasJustRunning_ = false;
			} else {
				if (timeDiff_ <= 2.5) {
					robot_->SetIntakeOutput(intakeMotorOutput_);
					wasJustRunning_ = true;
				} else {
					robot_->SetIntakeOutput(0.0);
					isDone_ = true;
					printf("Intake Done from TIMEOUT!\n");
				}
			}
		}
	}
//	robot_->SetIntakeOutput(intakeMotorOutput_);
}

bool IntakeCommand::IsDone() {
	return isDone_;
}

IntakeCommand::~IntakeCommand() {
	// TODO Auto-generated destructor stub
	Reset();
}

