/*
 * TestMode.cpp
 *
 *  Created on: Jan 14, 2018
 *      Author: Lynn D
 */

#include <Auto/Modes/TestMode.h>

TestMode::TestMode(RobotModel *robot) : AutoMode(robot){
	printf("In Test Mode\n");
//	driveStraightFirst_ = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, 5.0);
//	pivot_ = new PivotCommand(robot_, 90.0, true, navX_);
//	elevatorCommand_ = new ElevatorHeightCommand(robot_, 3.0);
}

void TestMode::CreateQueue(string gameData, AutoMode::AutoPositions pos) {
	string autoSequence = robot_->testMode_;
	QueueFromString(autoSequence);
//	driveStraightFirst_->SetNextCommand(pivot_);
//	pivot_->SetNextCommand(new WristCommand(robot_, 0.0));
//	firstCommand_ = new ParallelCommand(driveStraightFirst_, elevatorCommand_);
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

