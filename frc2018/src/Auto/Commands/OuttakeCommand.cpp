/*
 * OuttakeCommand.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: alisha
 */

#include <Auto/Commands/OuttakeCommand.h>

OuttakeCommand::OuttakeCommand(RobotModel *robot) : AutoCommand() {
	// TODO Auto-generated constructor stub
	robot_ = robot;
	isDone_ = false;
	outtakeMotorOutput_ = -0.7; //TODO TEST
	startTime_ = 0.0;
	deltaTime_ = 2.5; //TODO TEST
}

void OuttakeCommand::Init() {
	isDone_ = false;
	startTime_ = robot_->GetTime();
}

void OuttakeCommand::Reset() {
	isDone_ = true;
}

void OuttakeCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (robot_->GetTime() - startTime_ > deltaTime_){
		robot_->SetIntakeOutput(0.0);
		isDone_ = true;
	} else {
		robot_->SetIntakeOutput(outtakeMotorOutput_);
	}
}

bool OuttakeCommand::IsDone() {
	return isDone_;
}

OuttakeCommand::~OuttakeCommand() {
	// TODO Auto-generated destructor stub
	Reset();
}

