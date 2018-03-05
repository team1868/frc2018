/*
 * WristCommand.cpp
 *
 *  Created on: Mar 4, 2018
 *      Author: starr
 */

#include "Auto/Commands/WristCommand.h"

WristCommand::WristCommand(RobotModel *robot, bool wristUp) : AutoCommand() {
	robot_ = robot;
	wristUp_ = wristUp;
	isDone_ = false;
}

void WristCommand::Init() {}
void WristCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (wristUp_) {
		robot_->SetWristUp();
	} else {
		robot_->SetWristDown();
	}

	isDone_ = true;
}

bool WristCommand::IsDone() {
	return isDone_;
}

void WristCommand::Reset() {}

WristCommand::~WristCommand() {
}

