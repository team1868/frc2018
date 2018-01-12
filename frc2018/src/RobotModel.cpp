#include <RobotModel.h>
#include <math.h>

const double WHEEL_DIAMETER = 3.5 / 12.0; 			// TODO CHANGE
const double ENCODER_COUNT_PER_ROTATION = 256.0;
const int EDGES_PER_ENCODER_COUNT = 4;

RobotModel::RobotModel() {
	// Initializing ini
	pini_ = new Ini("home/lvuser/robot.ini");

	// Initializing timer
	timer_= new Timer();
	timer_->Start();

	// Initializing pdp
	pdp_ = new PowerDistributionPanel();

	// Initializing Encoders
	leftDriveEncoder_ = new Encoder(LEFT_DRIVE_ENCODER_A_PWM_PORT, LEFT_DRIVE_ENCODER_B_PWM_PORT, false);		// TODO check if true or false
	leftDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);

	rightDriveEncoder_ = new Encoder(RIGHT_DRIVE_ENCODER_A_PWM_PORT, RIGHT_DRIVE_ENCODER_B_PWM_PORT, true);		// TODO check if true or false
	rightDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);

	// Initializing Talons
	leftMaster_ = new WPI_TalonSRX(LEFT_DRIVE_MASTER_ID);
	rightMaster_ = new WPI_TalonSRX(RIGHT_DRIVE_MASTER_ID);
	leftSlave_ = new WPI_TalonSRX(LEFT_DRIVE_SLAVE_ID);
	rightSlave_ = new WPI_TalonSRX(RIGHT_DRIVE_SLAVE_ID);

	leftMaster_->Set(ControlMode::PercentOutput, 0.0);
	rightMaster_->Set(ControlMode::PercentOutput, 0.0);
	leftSlave_->Set(ControlMode::Follower, LEFT_DRIVE_MASTER_ID);
	rightSlave_->Set(ControlMode::Follower, RIGHT_DRIVE_MASTER_ID);

	// Initializing NavX
	navX_ = new AHRS(SPI::kMXP);
	Wait(1.0); // NavX takes a second to calibrate

	// Initializing pneumatics
	compressor_ = new Compressor(PNEUMATICS_CONTROL_MODULE_ID);
	gearShiftSolenoid_ = new DoubleSolenoid(GEAR_SHIFT_FORWARD_SOLENOID_PORT, GEAR_SHIFT_REVERSE_SOLENOID_PORT);
}

void RobotModel::ResetTimer() {
	timer_->Reset();
}

double RobotModel::GetTime() {
	return timer_->Get();
}

void RobotModel::SetDriveValues(RobotModel::Wheels wheel, double value) {
	switch(wheel) {
		case (kLeftWheels):
			leftMaster_->Set(value);
			break;
		case (kRightWheels):
			rightMaster_->Set(value);
			break;
		case (kAllWheels):
			rightMaster_->Set(value);
			leftMaster_->Set(value);
	}
}

void RobotModel::SetHighGear() {
	gearShiftSolenoid_->Set(DoubleSolenoid::kForward); // TODO Check if right
}

void RobotModel::SetLowGear() {
	gearShiftSolenoid_->Set(DoubleSolenoid::kReverse); // TODO Check if right
}

double RobotModel::GetDriveEncoderValue(RobotModel::Wheels wheel) { // TODO check if encoders are working
	switch(wheel) {
		case(kLeftWheels):
				return leftDriveEncoder_->Get();
		case(kRightWheels):
				return rightDriveEncoder_->Get();
		case(kAllWheels):
				return 0;
		default:
			return 0;
	}
}

double RobotModel::GetLeftDistance() {
	return leftDriveEncoder_->GetDistance();
}

double RobotModel::GetRightDistance() {
	return rightDriveEncoder_->GetDistance();
}

double RobotModel::GetNavXYaw() {
	return navX_->GetYaw();
}

void RobotModel::ZeroNavXYaw() {
	navX_->ZeroYaw();
}

void RobotModel::RefreshIni() {
	delete pini_;
	pini_ = new Ini("/home/lvuser/robot.ini");
}

RobotModel::~RobotModel() {
	// TODO Auto-generated destructor stub
}

