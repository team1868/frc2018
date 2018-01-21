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
	GetIniValues();

	anglePID_ = new PIDController(rPFac_, rIFac_, rDFac_, navXSource_, anglePIDOutput_);
	distancePID_ = new PIDController(dPFac_, dIFac_, dDFac_, talonEncoderSource_, distancePIDOutput_);

	// initializing number of times robot is on target
	numTimesOnTarget_ = 0;

}

void DriveStraightCommand::Init() {
	//robot_->SetPercentVBusDriveMode(); //Not included in RobotModel.h so we took it out

	isDone_ = false;

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;

	initialAngle_ = navXSource_->PIDGet();
	initialAvgDistance_ = talonEncoderSource_->PIDGet();
	desiredTotalAvgDistance_ = initialAvgDistance_ + desiredDistance_;

	anglePID_->SetSetpoint(initialAngle_);
	distancePID_->SetSetpoint(desiredTotalAvgDistance_);

	anglePID_->SetContinuous(false);
	distancePID_->SetContinuous(false);

	anglePID_->SetOutputRange(-0.2, 0.2); //LEAVING VALUES EMPTY BC THEY HAVE NOT BEEN FIGURED OUT YET
	distancePID_->SetOutputRange(-0.8, 0.8); //LEAVING VALUES EMPTY BC THEY HAVE NOT BEEN FIGURED OUT YET

	anglePID_->SetAbsoluteTolerance(2.0); //LEAVING VALUES EMPTY BC THEY HAVE NOT BEEN FIGURED OUT YET
	distancePID_->SetAbsoluteTolerance(2.5/12.0); //LEAVING VALUES EMPTY BC THEY HAVE NOT BEEN FIGURED OUT YET

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

	diffDriveTime_ = robot_->GetTime() - initialDriveTime_;
	if((anglePID_->OnTarget() && (distancePID_->OnTarget())) || (diffDriveTime_ > 10.0)) { //LEAVING AS 10.0 FOR NOW BC WE DON'T KNOW ACTUAL VALUES
		if(diffDriveTime_ > 10.0) { //LEAVING AS 10.0 FOR NOW BC WE DON'T KNOW ACTUAL VALUES
			printf("DRIVESTRAIGHT TIMED OUT!! :) \n");
		}
		numTimesOnTarget_++;
		if ((anglePID_->OnTarget() && (distancePID_->OnTarget())) && (numTimesOnTarget_ >= 3)) { // Ensure the robot is on target by recording 3 times
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

		rightMotorOutput_ = dOutput + rOutput;
		leftMotorOutput_ = dOutput - rOutput;

		double maxOutput = fmax(fabs(rightMotorOutput_), fabs(leftMotorOutput_));

		SmartDashboard::PutNumber("Angle Error", anglePID_->GetError());
		SmartDashboard::PutNumber("Angle Error Graph", anglePID_->GetError());
		SmartDashboard::PutNumber("DesiredAngle", initialAngle_);
		SmartDashboard::PutNumber("Encoder Error Feet", distancePID_->GetError());
		SmartDashboard::PutNumber("Encoder Error Feet Graph", distancePID_->GetError());
		SmartDashboard::PutNumber("Desired Total Feet", desiredTotalAvgDistance_);
	}

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftMotorOutput_);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightMotorOutput_);
}

bool DriveStraightCommand::IsDone() {
	return isDone_;
}

void DriveStraightCommand::GetIniValues() {
	rPFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "rPFac", 0.0);
	rIFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "rIFac", 0.0);
	rDFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "rDFac", 0.0);

	dPFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "dPFac", 0.0); //LEAVING AS 0.0 FOR NOW BC WE DON'T KNOW ACTUAL VALUES
	dIFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "dIFac", 0.0);
	dDFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "dDFac", 0.0);
}

DriveStraightCommand::~DriveStraightCommand() {

}


