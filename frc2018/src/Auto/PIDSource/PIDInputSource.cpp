#include <Auto/PIDSource/PIDInputSource.h>
#include "WPILib.h"

NavXPIDSource::NavXPIDSource(RobotModel *robot) {
	robot_ = robot;
	ResetAccumulatedYaw();
}

double NavXPIDSource::PIDGet() {
//	CalculateAccumulatedYaw();
	accumulatedYaw_ = robot_->GetNavXYaw();
	return accumulatedYaw_;
}

double NavXPIDSource::CalculateAccumulatedYaw() {
	lastYaw_ = currYaw_;
	currYaw_ = robot_->GetNavXYaw();
	deltaYaw_ = currYaw_ - lastYaw_;

	if (deltaYaw_ < -180) {
		accumulatedYaw_ += (180 - lastYaw_) + (180 + currYaw_);
	} else if (deltaYaw_ > 180) {
		accumulatedYaw_ -= (180 + lastYaw_) + (180 - currYaw_);
	} else {
		accumulatedYaw_ += deltaYaw_;
	}

	return accumulatedYaw_;
}

void NavXPIDSource::ResetAccumulatedYaw() {
	accumulatedYaw_ = 0.0;
	printf("darn\n");
	currYaw_ = robot_->GetNavXYaw();
	printf("finished resetting yaw\n");
	lastYaw_ = currYaw_;
	deltaYaw_ = 0.0;
}

NavXPIDSource::~NavXPIDSource() {

}
TalonEncoderPIDSource::TalonEncoderPIDSource(RobotModel* robot) {
	robot_ = robot;
	averageTalonDistance_ = 0.0;

}

double TalonEncoderPIDSource::PIDGet() {
	double leftDistance = robot_->GetLeftDistance();
	double rightDistance = robot_->GetRightDistance();

	averageTalonDistance_= (rightDistance + leftDistance) / 2;

	SmartDashboard::PutNumber("Left Distance", leftDistance);
	SmartDashboard::PutNumber("Right Distance", rightDistance);
	SmartDashboard::PutNumber("Average Distance", averageTalonDistance_);
	return averageTalonDistance_;

}

TalonEncoderPIDSource::~TalonEncoderPIDSource() {

}

ElevatorEncoderPIDSource::ElevatorEncoderPIDSource(RobotModel* robot) {
	robot_ = robot;
	elevatorHeight_ = 0.0;
}

double ElevatorEncoderPIDSource::PIDGet() {
	double leftDistance = robot_->GetLeftDistance();
	double rightDistance = robot_->GetRightDistance();

	elevatorHeight_ = robot_->GetElevatorHeight();

	return elevatorHeight_;
}

ElevatorEncoderPIDSource::~ElevatorEncoderPIDSource() {

}
