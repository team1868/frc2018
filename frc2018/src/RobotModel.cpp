#include <WPILib.h>
#include <RobotModel.h>

const double WHEEL_DIAMETER = 6.0 / 12.0; 			// TODO CHANGE
const double ENCODER_COUNT_PER_ROTATION = 256.0;
const int EDGES_PER_ENCODER_COUNT = 4;
const double ELEVATOR_DISTANCE_PER_PULSE = 1.0; // TODO FIGURE OUT ACTUAL VALUE

RobotModel::RobotModel() {
	// Initializing ini
	pini_ = new Ini("home/lvuser/robot.ini");
	// Initializing Pivot PID vals
	pivotPFac_ = 0.0;
	pivotIFac_ = 0.0;
	pivotDFac_ = 0.0;
	// Initializing DriveStraight PID vals
	driveDPFac_ = 0.0;
	driveDIFac_ = 0.0;
	driveDDFac_ = 0.0;
	driveRPFac_ = 0.0;
	driveRIFac_ = 0.0;
	driveRDFac_ = 0.0;
	// Initializing Elevator PID vals
	elevatorPFac_ = 0.0;
	elevatorIFac_ = 0.0;
	elevatorDFac_ = 0.0;

	driveTimeoutSec_ = 0.0; // TODO add to ini file
	pivotTimeoutSec_ = 0.0; // TODO add to ini file

	string cubeInSwitchL_ = ""; // TODO add to ini file
	string cubeInSwitchR_ = ""; // TODO add to ini file

	// Initializing timer
	timer_= new Timer();
	timer_->Start();

	// Initializing pdp
	pdp_ = new PowerDistributionPanel();

	// Initializing Encoders
	leftDriveEncoder_ = new Encoder(LEFT_DRIVE_ENCODER_A_PWM_PORT, LEFT_DRIVE_ENCODER_B_PWM_PORT, true);		// TODO check if true or false
	leftDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);

	rightDriveEncoder_ = new Encoder(RIGHT_DRIVE_ENCODER_A_PWM_PORT, RIGHT_DRIVE_ENCODER_B_PWM_PORT, false);
	rightDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);

	// Initializing Drive Talons
	isLeftInverted_ = false;
	leftMaster_ = new WPI_TalonSRX(LEFT_DRIVE_MASTER_ID);
	rightMaster_ = new WPI_TalonSRX(RIGHT_DRIVE_MASTER_ID);
	leftSlave_ = new WPI_VictorSPX(LEFT_DRIVE_SLAVE_ID);
	rightSlave_ = new WPI_VictorSPX(RIGHT_DRIVE_SLAVE_ID);

	rightMaster_->SetInverted(!isLeftInverted_);
	rightSlave_->SetInverted(!isLeftInverted_);
	leftMaster_->SetInverted(isLeftInverted_);
	leftSlave_->SetInverted(isLeftInverted_);

	leftMaster_->Set(ControlMode::PercentOutput, 0.0);
	rightMaster_->Set(ControlMode::PercentOutput, 0.0);
	leftSlave_->Set(ControlMode::Follower, LEFT_DRIVE_MASTER_ID);
	rightSlave_->Set(ControlMode::Follower, RIGHT_DRIVE_MASTER_ID);

	// Initializing NavX
	navX_ = new AHRS(SPI::kMXP);
//	navX_ = new AHRS(SerialPort::Port::kUSB);
	Wait(1.0); // NavX takes a second to calibrate

	// Initializing pneumatics
	compressor_ = new Compressor(PNEUMATICS_CONTROL_MODULE_ID);
//	gearShiftSolenoid_ = new DoubleSolenoid(GEAR_SHIFT_FORWARD_SOLENOID_PORT, GEAR_SHIFT_REVERSE_SOLENOID_PORT);

	leftIntakeMotor_ = new Victor(LEFT_INTAKE_MOTOR_PWM_PORT);
	rightIntakeMotor_ = new Victor(RIGHT_INTAKE_MOTOR_PWM_PORT);
	elevatorMotor_ = new Victor(ELEVATOR_MOTOR_PWM_PORT);

	elevatorEncoder_ = new Encoder(ELEVATOR_ENCODER_A_PWM_PORT, ELEVATOR_ENCODER_B_PWM_PORT, false);
	elevatorEncoder_->SetDistancePerPulse(ELEVATOR_DISTANCE_PER_PULSE);

	intakeSensor_ = new DigitalInput(INTAKE_SENSOR_PWM_PORT);

}

void RobotModel::ResetTimer() {
	timer_->Reset();
}

double RobotModel::GetTime() {
	return timer_->Get();
}

WPI_TalonSRX *RobotModel::GetTalon(Talons talon) {
	switch(talon) {
		case(kLeftMaster):
				return leftMaster_;
		case(kRightMaster):
				return rightMaster_;
		default:
			return NULL;
	}
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
//	gearShiftSolenoid_->Set(DoubleSolenoid::kForward); // TODO Check if right
}

void RobotModel::SetLowGear() {
//	gearShiftSolenoid_->Set(DoubleSolenoid::kReverse); // TODO Check if right
}

double RobotModel::GetLeftEncoderValue() {
	return leftDriveEncoder_->Get();
}

double RobotModel::GetRightEncoderValue() {
	return rightDriveEncoder_->Get();
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

double RobotModel::GetNavXPitch() {
	return navX_->GetPitch();
}

double RobotModel::GetNavXRoll() {
	return navX_->GetRoll();
}

void RobotModel::SetIntakeOutput(double output) {
	leftIntakeMotor_->Set(output);
	rightIntakeMotor_->Set(output);
}

void RobotModel::SetElevatorOutput(double output) {
	elevatorMotor_->Set(output);
}

double RobotModel::GetElevatorHeight() {
	return elevatorEncoder_->GetDistance();
}

bool RobotModel::GetCubeInIntake() {
	return intakeSensor_->Get();
}

Victor* RobotModel::GetElevatorMotor() {
	return elevatorMotor_;
}

void RobotModel::RefreshIni() {
	delete pini_;
	const char* usbPath = "insert path here"; // TODO fix
	if(FILE *file = fopen(usbPath, "r")) {
		fclose(file);
		pini_ = new Ini(usbPath);
	} else {
		pini_ = new Ini("/home/lvuser/robot.ini");
	}

	RefreshIniVals();
}

void RobotModel::RefreshIniVals() {
	pivotPFac_ = pini_->getf("PIVOT PID", "pFac", 0.0);
	pivotIFac_ = pini_->getf("PIVOT PID", "iFac", 0.0);
	pivotDFac_ = pini_->getf("PIVOT PID", "dFac", 0.0);
	pivotTimeoutSec_ = pini_->getf("PIVOT PID", "pivotTimeoutSec", 3.5);

	driveDPFac_ = pini_->getf("DRIVESTRAIGHT PID", "dPFac", 0.0);
	driveDIFac_ = pini_->getf("DRIVESTRAIGHT PID", "dIFac", 0.0);
	driveDDFac_ = pini_->getf("DRIVESTRAIGHT PID", "dDFac", 0.0);
	driveRPFac_ = pini_->getf("DRIVESTRAIGHT PID", "rPFac", 0.0);
	driveRIFac_ = pini_->getf("DRIVESTRAIGHT PID", "rIFac", 0.0);
	driveRDFac_ = pini_->getf("DRIVESTRAIGHT PID", "rDFac", 0.0);
	driveTimeoutSec_ = pini_->getf("DRIVESTRAIGHT PID", "driveTimeoutSec", 3);

	elevatorPFac_ = pini_->getf("ELEVATOR PID", "pFac", 0.0);
	elevatorIFac_ = pini_->getf("ELEVATOR PID", "iFac", 0.0);
	elevatorDFac_ = pini_->getf("ELEVATOR PID", "dFac", 0.0);

	cubeInSwitchL_ = pini_->gets("CUBE IN SWITCH", "cubeInSwitchL", "d10");
	cubeInSwitchR_ = pini_->gets("CUBE IN SWITCH", "cubeInSwitchR", "d10");
}

RobotModel::~RobotModel() {
	// TODO Auto-generated destructor stub
}

