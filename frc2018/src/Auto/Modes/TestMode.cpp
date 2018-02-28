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
	string autoSequence = "d10 t90";
	QueueFromString(autoSequence);
}

void TestMode::Init() {
	printf("Initializing Test mode\n");
	currentCommand_->Init();
}

TestMode::~TestMode() {
	// TODO Auto-generated destructor stub
}

