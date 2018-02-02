/*
 * ElevatorHeightCommand.cpp
 *
 *  Created on: Jan 29, 2018
 *      Author: alisha
 */

#include <Auto/Commands/ElevatorHeightCommand.h>

ElevatorHeightCommand::ElevatorHeightCommand(RobotModel *robot, double desiredHeight) {
	robot_ = robot;
	desiredHeight_ = desiredHeight;

	isDone_ = false;
	numTimesOnTarget_ = 0;

	ElevatorEncoderPIDSource *encoderPIDSource_ = new ElevatorEncoderPIDSource(robot_);
	Victor *elevatorPIDOutput_ = robot_->GetElevatorMotor();

	GetIniValues();

	elevatorHeightPID_ = new PIDController(pFac_, iFac_, dFac_, encoderPIDSource_,elevatorPIDOutput_);
	maxOutput_ = 0.8; // TODO Test
	tolerance_ = 1; // TODO CHANGE

	startTime_ = robot_->GetTime();
}

ElevatorHeightCommand::ElevatorHeightCommand(RobotModel *robot) {
	robot_ = robot;
	desiredHeight_ = 0.0;

	isDone_ = false;
	numTimesOnTarget_ = 0;

	ElevatorEncoderPIDSource *encoderPIDSource_ = new ElevatorEncoderPIDSource(robot_);
	Victor *elevatorPIDOutput_ = robot_->GetElevatorMotor();

	GetIniValues();

	elevatorHeightPID_ = new PIDController(pFac_, iFac_, dFac_, encoderPIDSource_,elevatorPIDOutput_);
	maxOutput_ = 0.8; // TODO Test
	tolerance_ = 1; // TODO CHANGE

	startTime_ = robot_->GetTime();
}

void ElevatorHeightCommand::Init() {
	GetIniValues();
	elevatorHeightPID_->SetPID(pFac_, iFac_, dFac_);

	elevatorHeightPID_->SetSetpoint(desiredHeight_);
	elevatorHeightPID_->SetOutputRange(-maxOutput_, maxOutput_);
	elevatorHeightPID_->SetAbsoluteTolerance(tolerance_);
	elevatorHeightPID_->Enable();

	isDone_ = false;
	numTimesOnTarget_ = 0;

	startTime_ = robot_->GetTime();
}

void ElevatorHeightCommand::Reset() {
	elevatorHeightPID_->Reset();
	elevatorHeightPID_->Disable();
	isDone_ = true;
}

void ElevatorHeightCommand::Update(double currTimeSec, double deltaTimeSec) {
	double timeDiff = robot_->GetTime() - startTime_;
	bool timeOut = (timeDiff > 3.5);								//test this value

	if (elevatorHeightPID_->OnTarget()) {
		numTimesOnTarget_++;
	} else {
		numTimesOnTarget_ = 0;
	}

	if ((elevatorHeightPID_->OnTarget() && numTimesOnTarget_ > 3) || timeOut) {
		elevatorHeightPID_->Reset();
		elevatorHeightPID_->Disable();
		isDone_ = true;
		robot_->SetDriveValues(RobotModel::kLeftWheels, 0.0);
		robot_->SetDriveValues(RobotModel::kRightWheels, 0.0);
		if (timeOut) {
			printf("FROM TIME OUT\n");
		}
	}
}

void ElevatorHeightCommand::SetSetpoint(double newHeight) {
	desiredHeight_ = newHeight;
}

bool ElevatorHeightCommand::IsDone() {
	return isDone_;
}

void ElevatorHeightCommand::GetIniValues() {
	pFac_ = robot_->elevatorPFac_;
	iFac_ = robot_->elevatorIFac_;
	dFac_ = robot_->elevatorDFac_;
}

ElevatorHeightCommand::~ElevatorHeightCommand() {
	Reset();
	delete(elevatorHeightPID_);
}

