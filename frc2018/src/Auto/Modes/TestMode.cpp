/*
 * TestMode.cpp
 *
 *  Created on: Jan 14, 2018
 *      Author: Lynn D
 */

#include <Auto/Modes/TestMode.h>

TestMode::TestMode(RobotModel *robot) : AutoMode(robot){
	printf("In Test Mode\n");
}

void TestMode::CreateQueue(string gameData, AutoMode::AutoPositions pos) {
	string autoSequence = robot_->testMode_;
	QueueFromString(autoSequence);
//	firstCommand_ = new WristCommand(robot_, 0.0);
//	firstCommand_->SetNextCommand(new DriveIntakeCubeCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_));
//	currentCommand_ = firstCommand_;
}

void TestMode::Init() {
	printf("Initializing Test mode\n");
	currentCommand_->Init();
	printf("Finished initializing\n");
}

TestMode::~TestMode() {
	// TODO Auto-generated destructor stub
}

