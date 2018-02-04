/*
 * SequenceMode.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: alisha
 */

#include <Auto/Modes/SequenceMode.h>

SequenceMode::SequenceMode(RobotModel *robot, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoder) : AutoMode(robot, navX, talonEncoder) {
	autoSequenceStr_ = robot_->pini_->gets("TEST MODE", "autoSequence", "");

	firstCommand_ = NULL;
	lastCommand_ = NULL;
}

void SequenceMode::CreateQueue() {
	firstCommand_ = NULL;
	lastCommand_ = NULL;
	RefreshIni();
	std::istringstream iss(autoSequenceStr_);
	char command;
	double value;
	while (!iss.eof()) {
		iss >> command >> value;
		printf("Command: %c, Value: %f\n", command, value);
		if (firstCommand_ == NULL) {
			if (command == 'p') {
				firstCommand_ = new PivotCommand(robot_, value, true, navX_);
			} else if (command == 'd') {
				firstCommand_ = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, value);
			}
			lastCommand_ = firstCommand_;
		} else {
			if (command == 'p') {
				lastCommand_->SetNextCommand(new PivotCommand(robot_, value, true, navX_));
				lastCommand_ = lastCommand_->GetNextCommand();
			} else if (command == 'd') {
				lastCommand_->SetNextCommand(new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, value));
				lastCommand_ = lastCommand_->GetNextCommand();
			}
		}
	}
}

void SequenceMode::Init() {
	RefreshIni();
	currentCommand_ = firstCommand_;
	currentCommand_->Init();
}

void SequenceMode::RefreshIni() {
	autoSequenceStr_ = robot_->pini_->gets("TEST MODE", "autoSequence", "");
}

SequenceMode::~SequenceMode() {
	// TODO Auto-generated destructor stub
}

