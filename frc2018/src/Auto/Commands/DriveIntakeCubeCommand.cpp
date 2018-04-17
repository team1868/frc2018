/*
 * DriveIntakeCubeCommand.cpp
 *
 *  Created on: Apr 7, 2018
 *      Author: starr
 */

#include <Auto/Commands/DriveIntakeCubeCommand.h>

DriveIntakeCubeCommand::DriveIntakeCubeCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot) : AutoCommand() {
	printf("In DriveIntakeCubeCommand Constructor\n");
	robot_ = robot;
	firstCommand_ = NULL;
	currentCommand_ = NULL;

	forwardCommand_ = new DriveStraightCommand(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot_, 2.0);
	backwardCommand_ = new DriveStraightCommand(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot_, -1.5);
	elevatorCommand_ = new ElevatorHeightCommand(robot_, 1.0);
	parallel2A_ = new ParallelCommand(backwardCommand_, elevatorCommand_);
	intakeFirst_ = new IntakeCommand(robot_, 0.8);
	forwardCommand_->SetNextCommand(parallel2A_);

	parallel1A_ = new ParallelCommand(forwardCommand_, intakeFirst_);


//	parallel2A_ = new ParallelCommand(backwardCommand_, elevatorCommand_);
//	parallel2B_ = new ParallelCommand(parallel2A_, intakeLast_);
//	parallel2B_ = new ParallelCommand(backwardCommand_, intakeLast_);

	wristUp_ = new WristCommand(robot_, 1.0);

	isDone_ = false;
}

void DriveIntakeCubeCommand::Init() {
	printf("In DriveIntakeCubeCommand Init\n");

//	parallel1A_->SetNextCommand(parallel2B_);
//	parallel2B_->SetNextCommand(wristUp_);

	firstCommand_ = parallel1A_;
	parallel1A_->SetNextCommand(wristUp_);
	currentCommand_ = firstCommand_;
	currentCommand_->Init();
	isDone_ = false;
}

void DriveIntakeCubeCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (currentCommand_ != NULL) {
		if (currentCommand_->IsDone()) {
			currentCommand_->Reset();
			currentCommand_ = currentCommand_->GetNextCommand();
			if (currentCommand_ != NULL) {
				currentCommand_->Init();
			}
		} else {
			currentCommand_->Update(currTimeSec, deltaTimeSec);
		}

	} else {
		printf("Drive Intake Cube Command Finished\n");
	}
}

bool DriveIntakeCubeCommand::IsDone() {
	return (currentCommand_ == NULL);
}

void DriveIntakeCubeCommand::Reset() {
	if (!IsDone()) {
		printf("Resetting current command\n");
		currentCommand_->Reset();
	}
	if (firstCommand_ != NULL) {
		currentCommand_ = firstCommand_;
		AutoCommand* nextCommand;
		while (currentCommand_ != NULL) {
			nextCommand = currentCommand_->GetNextCommand();
			delete(currentCommand_);
			currentCommand_ = NULL;
			currentCommand_ = nextCommand;
		}
		firstCommand_ = NULL;
	}

}

DriveIntakeCubeCommand::~DriveIntakeCubeCommand() {
	// TODO Auto-generated destructor stub
}

