/*
 * ElevatorBrakeCommand.cpp
 *
 *  Created on: Mar 6, 2018
 *      Author: alisha
 */

#include <Auto/Commands/ElevatorBrakeCommand.h>

ElevatorBrakeCommand::ElevatorBrakeCommand(RobotModel *robot, bool engageBrake) : AutoCommand() {
	robot_ = robot;
	engageBrake_ = engageBrake;
	isDone_ = false;
}

void ElevatorBrakeCommand::Init() {
	isDone_ = false;
}

void ElevatorBrakeCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (engageBrake_) {
		robot_->EngageBrake();
	} else {
		robot_->DisengageBrake();
	}

	isDone_ = true;
}

bool ElevatorBrakeCommand::IsDone() {
	return isDone_;
}

void ElevatorBrakeCommand::Reset() {
	isDone_ = true;
}

ElevatorBrakeCommand::~ElevatorBrakeCommand() {
	// TODO Auto-generated destructor stub
}

