#include <Auto/Commands/DriveStraightCommand.h>
#include "WPILib.h"

DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance) {
	robot_ = robot;

	navXSource_ = navXSource;
	talonEncoderSource_ = talonEncoderSource;

	anglePIDOutput_ = anglePIDOutput;
	distancePIDOutput_ = distancePIDOutput;

	initialAngle_ = navXSource_->PIDGet();
	initialAvgDistance_ = talonEncoderSource_->PIDGet();

	desiredDistance_ = desiredDistance;
	desiredTotalAvgDistance_ = initialAvgDistance_ + desiredDistance_;

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;
	isDone_ = false;
	initialDriveTime_ = robot_->GetTime();
	diffDriveTime_ = robot_->GetTime() - initialDriveTime_;

	// Setting up the PID controllers
	GetIniValues();
	anglePID_ = new PIDController(rPFac_, rIFac_, rDFac_, navXSource_, anglePIDOutput_);
	distancePID_ = new PIDController(dPFac_, dIFac_, dDFac_, talonEncoderSource_, distancePIDOutput_);

	rTolerance_ = 1.0;
	dTolerance_ = 1.0 / 12.0;

	rMaxOutput_ = 0.2;
	dMaxOutput_ = 0.7; // Currently for the KOP @ google (slipped a lot when higher)
	// initializing number of times robot is on target
	numTimesOnTarget_ = 0;

}

void DriveStraightCommand::Init() {
	isDone_ = false;

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;

	// Setting up PID vals again (in case they changed)
	GetIniValues();
	anglePID_->SetPID(rPFac_, rIFac_, rDFac_);
	distancePID_->SetPID(dPFac_, dIFac_, dDFac_);

	initialAngle_ = navXSource_->PIDGet();
	initialAvgDistance_ = talonEncoderSource_->PIDGet();
	desiredTotalAvgDistance_ = initialAvgDistance_ + desiredDistance_;

	anglePID_->SetSetpoint(initialAngle_);
	distancePID_->SetSetpoint(desiredTotalAvgDistance_);

	anglePID_->SetContinuous(false);
	distancePID_->SetContinuous(false);

	anglePID_->SetOutputRange(-rMaxOutput_, rMaxOutput_); //LEAVING VALUES EMPTY BC THEY HAVE NOT BEEN FIGURED OUT YET
	distancePID_->SetOutputRange(-dMaxOutput_, dMaxOutput_); //LEAVING VALUES EMPTY BC THEY HAVE NOT BEEN FIGURED OUT YET

	anglePID_->SetAbsoluteTolerance(rTolerance_); //LEAVING VALUES EMPTY BC THEY HAVE NOT BEEN FIGURED OUT YET
	distancePID_->SetAbsoluteTolerance(dTolerance_); //LEAVING VALUES EMPTY BC THEY HAVE NOT BEEN FIGURED OUT YET

	anglePID_->Enable();
	distancePID_->Enable();

	initialDriveTime_ = robot_->GetTime();

	numTimesOnTarget_ = 0;

	printf("Initial Right Distance: %f\n", robot_->GetDriveEncoderValue(RobotModel::kRightWheels));
	printf("Initial Left Distance: %f\n", robot_->GetDriveEncoderValue(RobotModel::kLeftWheels));
	printf("Initial Average Distance: %f\n", initialAvgDistance_);
	printf("Desired Distance: %f\n", desiredTotalAvgDistance_);

}

void DriveStraightCommand::Update(double currTimeSec, double deltaTimeSec) {
	SmartDashboard::PutNumber("Left Motor Output", leftMotorOutput_);
	SmartDashboard::PutNumber("Right Motor Output", rightMotorOutput_);
	SmartDashboard::PutNumber("Angle Error", anglePID_->GetError());
	SmartDashboard::PutNumber("Angle Error Graph", anglePID_->GetError());
	SmartDashboard::PutNumber("DesiredAngle", initialAngle_);
	SmartDashboard::PutNumber("Encoder Error Feet", distancePID_->GetError());
	SmartDashboard::PutNumber("Encoder Error Feet Graph", distancePID_->GetError());
	SmartDashboard::PutNumber("Desired Total Feet", desiredTotalAvgDistance_);

	diffDriveTime_ = robot_->GetTime() - initialDriveTime_;
	SmartDashboard::PutNumber("DriveStraight Time:", diffDriveTime_);
	if((anglePID_->OnTarget() && (distancePID_->OnTarget())) || (diffDriveTime_ > 3.0)) { //LEAVING AS 10.0 FOR NOW BC WE DON'T KNOW ACTUAL VALUES
		if(diffDriveTime_ > 3.0) { //LEAVING AS 10.0 FOR NOW BC WE DON'T KNOW ACTUAL VALUES
			printf("DRIVESTRAIGHT TIMED OUT!! :) \n");
			anglePID_->Reset();
			distancePID_->Reset();

			leftMotorOutput_ = 0.0;
			rightMotorOutput_ = 0.0;

			isDone_ = true;
		}
		numTimesOnTarget_++;
		SmartDashboard::PutNumber("Num times on target distance", numTimesOnTarget_);
		if ((distancePID_->OnTarget()) && (numTimesOnTarget_ > 3)) { // Ensure the robot is on target by recording 3 times
			printf("Final Left Distance: %f\n", robot_->GetDriveEncoderValue(RobotModel::kLeftWheels));
			printf("Final Right Distance: %f\n", robot_->GetDriveEncoderValue(RobotModel::kRightWheels));
			printf("Final Average Distance: %f\n", talonEncoderSource_->PIDGet());
			anglePID_->Reset();
			distancePID_->Reset();

			leftMotorOutput_ = 0.0;
			rightMotorOutput_ = 0.0;

			isDone_ = true;
		}
	} else {
		double dOutput = distancePIDOutput_->GetPIDOutput();
		double rOutput = anglePIDOutput_->GetPIDOutput();

		rightMotorOutput_ = dOutput - rOutput;
		leftMotorOutput_ = dOutput + rOutput;

//		double maxOutput = fmax(fabs(rightMotorOutput_), fabs(leftMotorOutput_));
	}

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftMotorOutput_);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightMotorOutput_);
}

bool DriveStraightCommand::IsDone() {
	return isDone_;
}

void DriveStraightCommand::Reset() {
	anglePID_->Disable();
	distancePID_->Disable();

	anglePID_->Reset();
	distancePID_->Reset();
	isDone_ = true;
}

void DriveStraightCommand::GetIniValues() { // Ini values are refreshed at the start of auto
	printf("Changed ini values\n");
	dPFac_ = robot_->driveDPFac_;
	dIFac_ = robot_->driveDIFac_;
	dDFac_ = robot_->driveDDFac_;

	rPFac_ = robot_->driveRPFac_;
	rIFac_ = robot_->driveRIFac_;
	rDFac_ = robot_->driveRDFac_;
	printf("DRIVESTRAIGHT COMMAND DRIVE p: %f, i: %f, d: %f\n", dPFac_, dIFac_, dDFac_);
	printf("DRIVESTRAIGHT COMMAND ANGLE p: %f, i: %f, d: %f\n", rPFac_, rIFac_, rDFac_);
}

DriveStraightCommand::~DriveStraightCommand() {

}


