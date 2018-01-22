/*
 * TestMode.cpp
 *
 *  Created on: Jan 14, 2018
 *      Author: Lynn D
 */

#include <Auto/Modes/TestMode.h>

TestMode::TestMode(RobotModel *robot, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoder) {
	robot_ = robot;
	navX_ = navX;
	talonEncoder_ = talonEncoder;

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	driveStraightFirst_ = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, 8.0);
	pivot_ = new PivotCommand(robot_, -90, false, navX_);
	driveStraightSecond_ = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, 3.0);
	firstCommand_ = NULL;
}

void TestMode::CreateQueue() {
	firstCommand_ = driveStraightFirst_; // change to drivestraight when ready to test
	driveStraightFirst_->SetNextCommand(pivot_);
	pivot_->SetNextCommand(driveStraightSecond_);
}

void TestMode::Init() {
	currentCommand_ = firstCommand_;
	currentCommand_->Init();
}

void TestMode::RefreshIni() {

}

TestMode::~TestMode() {
	// TODO Auto-generated destructor stub
}

