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

	driveStraightFirst_ = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, 5.0);
	pivot_ = new PivotCommand(robot_, -90, true, navX_);
	driveStraightSecond_ = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, 5.0);
	pivotSecond_ = new PivotCommand(robot_, -180, true, navX_);
	driveStraightThird_ = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, 5.0);
	pivotThird_ = new PivotCommand(robot_, 90, true, navX_);
	driveStraightFourth_ = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, 5.0);
	pivotFourth_ = new PivotCommand(robot_, 0, true, navX_);

	firstCommand_ = NULL;
}

void TestMode::CreateQueue() {
//	firstCommand_ = pivot_;
	firstCommand_ = driveStraightFirst_; // change to drivestraight when ready to test
	driveStraightFirst_->SetNextCommand(pivot_);
	pivot_->SetNextCommand(driveStraightSecond_);
	driveStraightSecond_->SetNextCommand(pivotSecond_);
	pivotSecond_->SetNextCommand(driveStraightThird_);
	driveStraightThird_->SetNextCommand(pivotThird_);
	pivotThird_->SetNextCommand(driveStraightFourth_);
	driveStraightFourth_->SetNextCommand(pivotFourth_);
}

void TestMode::Init() {
	RefreshIni();
	currentCommand_ = firstCommand_;
	currentCommand_->Init();
}

void TestMode::RefreshIni() {

}

TestMode::~TestMode() {
	// TODO Auto-generated destructor stub
}

