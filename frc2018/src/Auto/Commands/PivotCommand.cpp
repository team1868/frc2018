#include "Auto/Commands/PivotCommand.h"

PivotPIDTalonOutput::PivotPIDTalonOutput(RobotModel *robot){
		robot_ = robot;
		output_ = 0.0;
}

void PivotPIDTalonOutput::PIDWrite(double myOutput){
	output_ = myOutput;

	robot_->SetDriveValues(RobotModel::kLeftWheels, output_);
	robot_->SetDriveValues(RobotModel::kRightWheels, -output_);
	SmartDashboard::PutNumber("left output", output_);
	SmartDashboard::PutNumber("right output", -output_);
}

double PivotPIDTalonOutput::GetOutput() {
	return output_;
}

PivotPIDTalonOutput::~PivotPIDTalonOutput(){
}

PivotCommand::PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavXPIDSource* navXSource) {

	navXSource_ = navXSource;

	initYaw_ = navXSource_->CalculateAccumulatedYaw();

	if (isAbsolutePosition){
		desiredDeltaAngle_ = CalculateDeltaAngle(desiredAngle);
	} else {
		desiredDeltaAngle_ = desiredAngle;
	}

	isDone_ = false;
	robot_ = robot;
	talonOutput_ = new PivotPIDTalonOutput(robot_);

	pivotCommandStartTime_ = robot_->GetTime();
	GetIniValues();
	pivotPID_ = new PIDController(pFac_, iFac_, dFac_, navXSource_, talonOutput_);

	numTimesOnTarget_ = 0;

}

void PivotCommand::GetIniValues() {

	pFac_ = robot_->pini_->getf("PIVOT PID", "pFac", 0.0);
	iFac_ = robot_->pini_->getf("PIVOT PID", "iFac", 0.0);
	dFac_ = robot_->pini_->getf("PIVOT PID", "dFac", 0.0);
	minDrivePivotOutput_ = robot_->pini_->getf("PIVOT PID", "minDrivePivotOutput", 0.0);
	printf("PIVOT COMMAND p: %f, i: %f, d: %f\n", pFac_, iFac_, dFac_);
}

void PivotCommand::Init() {
	//Profiler profiler(robot_, "Pivot Init");
	initYaw_ = navXSource_->PIDGet();

	pivotPID_->SetSetpoint(initYaw_ + desiredDeltaAngle_);
	pivotPID_->SetContinuous(false);
	pivotPID_->SetOutputRange(-0.8, 0.8);     //adjust for 2018
	pivotPID_->SetAbsoluteTolerance(1.5);	 //adjust for 2018
	pivotPID_->Enable();

	printf("Initial NavX Angle: %f\n", initYaw_);
	printf("Desired NavX Angle: %f\n", initYaw_ + desiredDeltaAngle_);

	isDone_ = false;
	numTimesOnTarget_ = 0;
	pivotCommandStartTime_ = robot_->GetTime();
}

void PivotCommand::Reset() {
	pivotPID_->Reset();
	pivotPID_->Disable();
	isDone_ = true;
	free(pivotPID_);
	printf("DONE FROM RESET \n");
}

void PivotCommand::Update(double currTimeSec, double deltaTimeSec) {

	SmartDashboard::PutNumber("Pivot Error", pivotPID_->GetError());
	SmartDashboard::PutNumber("Setpoint", pivotPID_->GetSetpoint());
	SmartDashboard::PutNumber("Delta setpoint", pivotPID_->GetDeltaSetpoint());

	double timeDiff = robot_->GetTime() - pivotCommandStartTime_;
	bool timeOut = (timeDiff > 2.5);								//test this value

//	SmartDashboard::PutBoolean("Timed out", timeOut);
	SmartDashboard::PutNumber("Pivot time diff", timeDiff);

	if (pivotPID_->OnTarget()) {
			numTimesOnTarget_++;
		} else {
			numTimesOnTarget_ = 0;
		}
		if ((pivotPID_->OnTarget() && numTimesOnTarget_ > 3) || timeOut) {
			printf("Final NavX Angle: %f\n", navXSource_->PIDGet());
			printf("Angle NavX Error %f\n", pivotPID_->GetAvgError());
			pivotPID_->Reset();
			pivotPID_->Disable();
			isDone_ = true;
			printf("PIVOT IS DONE \n");
			if (timeOut) {
				printf("FROM TIME OUT\n");
			}
		}
	}

bool PivotCommand::IsDone() {
	return isDone_;
}

double PivotCommand::CalculateDeltaAngle(double desiredAngle) {
	double currYaw = fmod(initYaw_, 360.0);
	return desiredAngle - currYaw;
}

PivotCommand::~PivotCommand() {
	pivotPID_->Reset();
	pivotPID_->Disable();
	isDone_ = true;
	free(pivotPID_);
	printf("IS DONE FROM DECONSTRUCTOR\n");
}
