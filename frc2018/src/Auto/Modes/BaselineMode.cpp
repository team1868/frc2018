/*
 * BaselineMode.cpp
 *
 *  Created on: Feb 10, 2018
 *      Author: starr
 */

#include <Auto/Modes/BaselineMode.h>

BaselineMode::BaselineMode(RobotModel *robot) : AutoMode(robot) {
	printf("constructing baseline mode\n");

}

void BaselineMode::CreateQueue(string gameData, AutoMode::AutoPositions pos) {
//	currentCommand_ = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, 10.0);
	QueueFromString("d10.0");
}

void BaselineMode::Init() {
	currentCommand_->Init();
}

BaselineMode::~BaselineMode() {

}

