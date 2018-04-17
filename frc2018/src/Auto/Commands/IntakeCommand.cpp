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
	printf("%f Starting chicken tenders timeout with intake\n", startTime_);
}

void IntakeCommand::Reset() {
	isDone_ = true;

}

void IntakeCommand::Update(double currTimeSec, double deltaTimeSec) {
	timeDiff_ = robot_->GetTime() - startTime_;
	if (robot_->GetCubeInIntake()){
		robot_->SetIntakeOutput(0.0);
		isDone_ = true;
		printf("%f Intake Done! time diff: %f\n", robot_->GetTime(), timeDiff_);
	} else {
//		robot_->SetIntakeOutput(intakeMotorOutput_);
		if (timeDiff_ <= 0.7) {
			robot_->SetIntakeOutput(intakeMotorOutput_);
		} else {
			if (wasJustRunning_ && timeDiff_ <= 1.2) {
				robot_->SetIntakeOutput(-intakeMotorOutput_);
				wasJustRunning_ = false;
			} else {
				if (timeDiff_ <= 1.8) {
					robot_->SetIntakeOutput(intakeMotorOutput_);
					wasJustRunning_ = true;
				} else {
					robot_->SetIntakeOutput(0.0);
					isDone_ = true;
					printf("Intake Done from TIMEOUT! with time diff %f\n", timeDiff_);
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

