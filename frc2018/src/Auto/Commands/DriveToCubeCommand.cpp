/*
 * PivotToCubeCommand.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: Grace
 */

#include <Auto/Commands/DriveToCubeCommand.h>

using namespace std;

DriveToCubeCommand::DriveToCubeCommand(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource, AnglePIDOutput *angleOutput, DistancePIDOutput *distanceOutput) : AutoCommand(){
	printf("in beginning of DRIVE to cube command\n");

	robot_ = robot;
	navXSource_ = navXSource;
	talonSource_ = talonSource;

	angleOutput_ = angleOutput;
	distanceOutput_ = distanceOutput;

	currentCommand_ = NULL;
	driveToCubeCommand_ = new PivotToCubeCommand(robot_, navXSource_, talonSource_, true);
	intakingCommand_ = new DriveIntakeCubeCommand(navXSource_, talonSource_, angleOutput_, distanceOutput_, robot_);

	isDone_ = false;

	printf("in DRIVE to cube command constructor\n");
}

void DriveToCubeCommand::Init() {
	printf("in DRIVE to cube command init\n");
	currentCommand_ = driveToCubeCommand_;
	driveToCubeCommand_->SetNextCommand(intakingCommand_);
}

void DriveToCubeCommand::Update(double currTimeSec, double deltaTimeSec) {
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
		printf("Drive To Cube Command Finished\n");
	}
}

bool DriveToCubeCommand::IsDone() {
	return (currentCommand_ == NULL);
}

void DriveToCubeCommand::Reset() {
	if (!IsDone()) {
		printf("Resetting current command\n");
		currentCommand_->Reset();
	}
	if (driveToCubeCommand_ != NULL) {
		currentCommand_ = driveToCubeCommand_;
		AutoCommand* nextCommand;
		while (currentCommand_ != NULL) {
			nextCommand = currentCommand_->GetNextCommand();
			delete(currentCommand_);
			currentCommand_ = NULL;
			currentCommand_ = nextCommand;
		}
		driveToCubeCommand_ = NULL;
	}
}

DriveToCubeCommand::~DriveToCubeCommand() {
	Reset();
	printf("IS DONE FROM DECONSTRUCTOR\n");
}


