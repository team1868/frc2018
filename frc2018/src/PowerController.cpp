/*
 * PowerController.cpp
 *
 *  Created on: Mar 6, 2018
 *      Author: Grace
 */

#include <PowerController.h>
#include <WPILib.h>
#include <RobotModel.h>

PowerController::PowerController(RobotModel *robot, ControlBoard *humanControl) {
	robot_ = robot;
	humanControl_ = humanControl;
	totalCurrent_ = 0;
	totalVoltage_ = 0;

	size = 5;	// used to find average of 5 readings of current

	std::vector<double> pastLeftCurr(size, 0);
	std::vector<double> pastRightCurr(size, 0);
	std::vector<double> pastIntakeCurr(size, 0);
	std::vector<double> pastCompCurr(size, 0);
	std::vector<double> pastRioCurr(size, 0);

	avgLeftCurr_ = 0;
	avgRightCurr_ = 0;
	avgIntakeCurr_ = 0;
	avgCompCurr_ = 0;
	avgRioCurr_ = 0;

	driveCurrentLimit_ = 0;
	intakeCurrentLimit_ = 0;
	totalCurrentLimit_ = 0;
	voltageFloor_ = 0;
	pressureFloor_ = 0;

	driveWeight_ = 0.8 * (totalVoltage_ / 13);
	compWeight_ = 0;
	intakeWeight_ = 0;

	compWeight_ = 0.5 * (totalVoltage_ / 13) * (pressureFloor_ / robot->GetPressureSensorVal());
	intakeWeight_ = 0.6 * (totalVoltage_ / 13) * (robot_->GetIntakeMotorSpeed() * robot_->GetOuttakeMotorSpeed());
}

void PowerController::Update(double currTimeSec, double deltaTimeSec) {
	pastLeftCurr_.insert(pastLeftCurr_.begin(),
			(robot_->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN) +
			robot_->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN)) / 2); // putting average current draw of motorA and motorB into arr
	pastRightCurr_.insert(pastRightCurr_.begin(),
			(robot_->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN) +
					robot_->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN)) / 2);
	pastIntakeCurr_.insert(pastIntakeCurr_.begin(), robot_->GetCurrent(RIGHT_INTAKE_MOTOR_PDP_CHAN));
	pastCompCurr_.insert(pastCompCurr_.begin(), robot_->GetCompressorCurrent());
	pastRioCurr_.insert(pastRioCurr_.begin(), robot_->GetRIOCurrent());

	pastLeftCurr_.pop_back();	// taking out previous current
	pastRightCurr_.pop_back();
	pastIntakeCurr_.pop_back();
	pastCompCurr_.pop_back();
	pastRioCurr_.pop_back();

	avgLeftCurr_ = GetAverage(pastLeftCurr_);		// getting average of five current readings
	avgRightCurr_ = GetAverage(pastRightCurr_);
	avgIntakeCurr_ = GetAverage(pastIntakeCurr_);
	avgCompCurr_ = GetAverage(pastCompCurr_);
	avgRioCurr_ = GetAverage(pastRioCurr_);


	totalCurrent_ = avgCompCurr_ + avgLeftCurr_ + avgRightCurr_ + avgIntakeCurr_
			+ avgRioCurr_;
	totalVoltage_ = robot_->GetVoltage();

	driveCurrentLimit_ *= (totalVoltage_ / 12);
	intakeCurrentLimit_ *= (totalVoltage_ / 12);

	driveWeight_ = 0.8 * (totalVoltage_ / 12);
	compWeight_ = 0.5 * (totalVoltage_ / 12) * (pressureFloor_ / robot_->GetPressureSensorVal());
	intakeWeight_ = 0.6 * (totalVoltage_ / 12) *
		(robot_->GetIntakeMotorSpeed() * robot_->GetOuttakeMotorSpeed());
	// todo add outtake motors

	if (IsBatteryLow()) {
		printf("battery low");
	}

//	if (humanControl_->GetPowerBudgetDesired()) {
		if (totalVoltage_ < (voltageFloor_ + 2) ) {
			LimitSingle();
			PriorityScale();
		}
//	}
}

bool PowerController::IsBatteryLow() {

	if ((fabs(robot_->GetVoltage() - voltageFloor_) < 1) && totalCurrent_ < 20) {
		// todo 20 is an arbitrary value, test
		return true;
	} else {
		return false;
	}
}

void PowerController::LimitSingle() {
	// linear regression model of current vs speed:
	// current = 47.4 * speed - 4.84 changes depending on BATTERY VOLTAGE!!!!!axqwaz
	// todo scale drivecurrentlimit as voltage decreases
	double diffCurr = avgLeftCurr_ - driveCurrentLimit_;
	double scaledSpeed =  (robot_->GetWheelSpeed(RobotModel::kLeftWheels)
		- (diffCurr + 4.84) / 47.4) * totalVoltage_ / 13;
	if (diffCurr >= 0) {
		robot_->SetDriveValues(RobotModel::kLeftWheels, scaledSpeed);
	}

	diffCurr = avgRightCurr_ - driveCurrentLimit_;
	scaledSpeed =  (robot_->GetWheelSpeed(RobotModel::kRightWheels)
		- (diffCurr + 4.84) / 47.4) * totalVoltage_ / 13;
	if (diffCurr >= 0) {
		robot_->SetDriveValues(RobotModel::kRightWheels, scaledSpeed);
	}

	diffCurr = avgIntakeCurr_ - intakeCurrentLimit_;
	// scaledSpeed = robot->GetIntakeMotorSpeed() -
	// todo add scaling for intake motor and outtake motor, compressor, roborio
}

void PowerController::PriorityScale() {
	double roboRIORatio = avgRioCurr_ / totalCurrent_;

	double diffRatio = totalCurrent_ / totalCurrentLimit_;


/*
	double leftDriveRatio = avgLeftCurr / totalCurrent;
	double rightDriveRatio = avgRightCurr / totalCurrent;
	double compressorRatio = avgCompCurr / totalCurrent;
	double intakeRatio = avgIntakeCurr / totalCurrent
	*/
	if (diffRatio >= 1) {
// todo test total current values
		double adjLeftWheelSpeed = ((1 / diffRatio) - roboRIORatio) * robot_->GetWheelSpeed(RobotModel::kLeftWheels);
		double adjRightWheelSpeed = ((1 / diffRatio) - roboRIORatio) * robot_->GetWheelSpeed(RobotModel::kRightWheels);
		robot_->SetDriveValues(RobotModel::kLeftWheels, driveWeight_ * adjLeftWheelSpeed);
		robot_->SetDriveValues(RobotModel::kRightWheels, driveWeight_ * adjRightWheelSpeed);
		// todo 0.7 is completely arbitrary -- test

		double adjIntakeSpeed =  ((1 / diffRatio) - roboRIORatio) * robot_->GetIntakeMotorSpeed();
		robot_->SetIntakeOutput(intakeWeight_ * adjIntakeSpeed);
		CompressorCut(); // todo incorporate compressor weight
	}
}

void PowerController::CompressorCut() {
	double compressorRatio = avgCompCurr_ / totalCurrent_;
	if ((totalCurrent_ > totalCurrentLimit_) &&
			(robot_->GetPressureSensorVal() > pressureFloor_)) {
		robot_->StopCompressor();
	}
}

double PowerController::GetAverage(std::vector<double> v) {
	double avg = 0;
	for (size_t i = 1; i < (v.size() + 1); i++) {
		avg += (v[i] - avg) / i;
	}
	return avg;
}


void PowerController::Reset() {
	totalCurrent_ = 0;
	totalVoltage_ = 0;
	avgLeftCurr_ = 0;
	avgRightCurr_ = 0;
	avgIntakeCurr_ = 0;
	avgCompCurr_ = 0;
	avgRioCurr_ = 0;
}

PowerController::~PowerController() {

}

