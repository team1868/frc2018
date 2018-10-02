/*
 * ElevatorHeightCommand.cpp
 *
 *  Created on: Jan 29, 2018
 *      Author: alisha
 */

#include <Auto/Commands/ElevatorHeightCommand.h>
#include "WPILib.h"

ElevatorHeightCommand::ElevatorHeightCommand(RobotModel *robot, double desiredHeight) : AutoCommand(){
	robot_ = robot;
	desiredHeight_ = desiredHeight;

	isDone_ = false;
	numTimesOnTarget_ = 0;

	ElevatorEncoderPIDSource *encoderPIDSource_ = new ElevatorEncoderPIDSource(robot_);
	Victor *elevatorPIDOutput_ = robot_->GetElevatorMotor();

	GetIniValues();

	elevatorHeightPID_ = new PIDController(pFac_, iFac_, dFac_, encoderPIDSource_,elevatorPIDOutput_);
	lastElevatorOutput_ = 0.0;
	maxElevatorRate_ = robot_->elevatorRampRate_;
	tolerance_ = 0.1; // TODO CHANGE

	startTime_ = robot_->GetTime();

	elevatorCurrentLimit_ = 40.0;
}

ElevatorHeightCommand::ElevatorHeightCommand(RobotModel *robot) {
	robot_ = robot;
	desiredHeight_ = 0.0;

	isDone_ = false;
	numTimesOnTarget_ = 0;

	Encoder *encoderPIDSource_ = robot_->GetElevatorEncoder();
	Victor *elevatorPIDOutput_ = robot_->GetElevatorMotor();

	GetIniValues();

	elevatorHeightPID_ = new PIDController(pFac_, iFac_, dFac_, encoderPIDSource_,elevatorPIDOutput_);
	tolerance_ = 1; // TODO CHANGE

	startTime_ = robot_->GetTime();
	actualMaxOutput_ = robot_->elevatorTallMaxOutput_;
}

void ElevatorHeightCommand::Init() {
	GetIniValues();
	elevatorHeightPID_->SetPID(pFac_, iFac_, dFac_);

	elevatorHeightPID_->SetSetpoint(desiredHeight_);
	elevatorHeightPID_->SetOutputRange(-maxOutput_, maxOutput_);
	elevatorHeightPID_->SetAbsoluteTolerance(tolerance_);
	elevatorHeightPID_->Enable();

	if (desiredHeight_ <= 1.0) {
		actualMaxOutput_ = robot_->elevatorMaxOutput_;
	} else if (fabs(elevatorHeightPID_->GetError()) > 3.0) {
		actualMaxOutput_ = robot_->elevatorTallMaxOutput_;
		printf("DOING SLOW STUFF :(");
	} else {
		actualMaxOutput_ = robot_->elevatorMaxOutput_;
	}

	isDone_ = false;
	numTimesOnTarget_ = 0;

	startTime_ = robot_->GetTime();
	printf("%f ELEVATOR PID VALUES: P: %f, I: %f, D: %f, actualMaxOutput: %f, Elevator PID error: %f\n", robot_->GetTime(), pFac_, iFac_, dFac_, actualMaxOutput_, fabs(elevatorHeightPID_->GetError()));
	robot_->DisengageBrake();
}

void ElevatorHeightCommand::Reset() {
	if (elevatorHeightPID_ != NULL) {
		printf("Elevator error %f\n", fabs(elevatorHeightPID_->GetError()));

		elevatorHeightPID_->Reset();
		elevatorHeightPID_->Disable();

		delete(elevatorHeightPID_);
		elevatorHeightPID_ = NULL;
	}
	isDone_ = true;
}

void ElevatorHeightCommand::Update(double currTimeSec, double deltaTimeSec) {
	SmartDashboard::PutNumber("Elevator Height", robot_->GetElevatorEncoder()->GetDistance());
//	SmartDashboard::PutNumber("Elevator Height Error", elevatorHeightPID_->GetError());
//	SmartDashboard::PutNumber("Elevator Current", robot_->GetElevatorCurrent());
//	SmartDashboard::PutNumber("Elevator Motor Output", elevatorPIDOutput_->Get());
	double timeDiff = robot_->GetTime() - startTime_;
//	SmartDashboard::PutNumber("Elevator Time Diff", timeDiff);
	bool timeOut = (timeDiff > 5.0);								//test this value

	//teleop version of ramp up (actually works)
	//if (elevatorUpOutput_ < elevatorMaxOutput_) {
	//	elevatorUpOutput_ *= elevatorRamp_;
	//}


	if (maxOutput_ < actualMaxOutput_) {
		maxOutput_ *= maxElevatorRate_;
		if (maxOutput_ > actualMaxOutput_) {
			maxOutput_ = actualMaxOutput_;
		}
		elevatorHeightPID_->SetOutputRange(-maxOutput_, maxOutput_);
		printf("elevator MAX output: %f\n", maxOutput_);
	}
//	} else if (maxOutput_ < robot_->elevatorMaxOutput_) {
//		maxOutput_ *= maxElevatorRate_;
//		 if (maxOutput_ > robot_->elevatorMaxOutput_) {
//			maxOutput_ = robot_->elevatorMaxOutput_;
//		}
//		elevatorHeightPID_->SetOutputRange(-maxOutput_, maxOutput_);
//	}

	if (elevatorHeightPID_->OnTarget()) {
		numTimesOnTarget_++;
	} else {
		numTimesOnTarget_ = 0;
	}

	if ((elevatorHeightPID_->OnTarget() && numTimesOnTarget_ > 3) || timeOut || robot_->GetElevatorCurrent() > elevatorCurrentLimit_) {
		isDone_ = true;
		robot_->EngageBrake();
		if (timeOut) {
			printf("%f Elevator FROM TIME OUT\n", robot_->GetTime());
		}
		printf("%f Elevator Command Done\n", robot_->GetTime());

		Reset();
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
	maxOutput_ = robot_->elevatorOutput_;
}

ElevatorHeightCommand::~ElevatorHeightCommand() {
	Reset();
	delete(elevatorHeightPID_);
}

