#include <WPILib.h>
#include <RobotModel.h>

const double WHEEL_DIAMETER = 6.0 / 12.0; 			// TODO CHANGE
const double ENCODER_COUNT_PER_ROTATION = 256.0;
const int EDGES_PER_ENCODER_COUNT = 4;
const double ELEVATOR_DISTANCE_PER_PULSE = (43.25 / 12) / 1165; // VALUE FROM PRACTICE BOT in feet

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

	string cubeInSwitchL_ = "";
	string cubeInSwitchR_ = "";

	autoPos_ = 0; //TODO add to ini file
	autoMode_ = 0; //TODO add to ini file

	intakeMotorOutput_ = 0.0;
	outtakeMotorOutput_ = 0.0;

	// Initializing timer
	timer_= new Timer();
	timer_->Start();

	// Initializing pdp
	pdp_ = new PowerDistributionPanel();

	// Initializing Encoders
	leftDriveEncoder_ = new Encoder(LEFT_DRIVE_ENCODER_YELLOW_PWM_PORT, LEFT_DRIVE_ENCODER_RED_PWM_PORT, true);		// TODO check if true or false
	leftDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);
	leftDriveEncoder_->SetReverseDirection(true);

	rightDriveEncoder_ = new Encoder(RIGHT_DRIVE_ENCODER_YELLOW_PWM_PORT, RIGHT_DRIVE_ENCODER_RED_PWM_PORT, false);
	rightDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);
	rightDriveEncoder_->SetReverseDirection(false);

	// Initializing Drive Talons
	isLeftInverted_ = true;	// FOR PRACT
//	isLeftInverted_ = false; // For KOP

	leftMaster_ = new WPI_TalonSRX(LEFT_DRIVE_MASTER_ID);
	rightMaster_ = new WPI_TalonSRX(RIGHT_DRIVE_MASTER_ID);
	leftSlave_ = new WPI_VictorSPX(LEFT_DRIVE_SLAVE_ID);
	rightSlave_ = new WPI_VictorSPX(RIGHT_DRIVE_SLAVE_ID);

	// Setting talon control modes and slaves
	leftMaster_->Set(ControlMode::PercentOutput, 0.0);
	rightMaster_->Set(ControlMode::PercentOutput, 0.0);
	leftSlave_->Follow(*leftMaster_);
	rightSlave_->Follow(*rightMaster_);

	// Setting Inversions
	rightMaster_->SetInverted(!isLeftInverted_);
	rightSlave_->SetInverted(!isLeftInverted_);
	leftMaster_->SetInverted(isLeftInverted_);
	leftSlave_->SetInverted(isLeftInverted_);

	// Initializing NavX
	navX_ = new AHRS(SPI::kMXP);
//	navX_ = new AHRS(SerialPort::kUSB);
	Wait(1.0); // NavX takes a second to calibrate

	// Initializing pneumatics
	compressor_ = new Compressor(PNEUMATICS_CONTROL_MODULE_ID);
	gearShiftSolenoid_ = new DoubleSolenoid(GEAR_SHIFT_FORWARD_SOLENOID_PORT, GEAR_SHIFT_REVERSE_SOLENOID_PORT);

	leftIntakeMotor_ = new Victor(LEFT_INTAKE_MOTOR_PWM_PORT);
	rightIntakeMotor_ = new Victor(RIGHT_INTAKE_MOTOR_PWM_PORT);
	elevatorMotor_ = new Victor(ELEVATOR_MOTOR_PWM_PORT);
	elevatorMotor_->SetInverted(true);

	elevatorEncoder_ = new Encoder(ELEVATOR_ENCODER_YELLOW_PWM_PORT, ELEVATOR_ENCODER_RED_PWM_PORT, false);
	elevatorEncoder_->SetDistancePerPulse(ELEVATOR_DISTANCE_PER_PULSE);

	rampLMotor_ = new Victor(RAMP_L_MOTOR_PWM_PORT);
	rampRMotor_ = new Victor(RAMP_R_MOTOR_PWM_PORT);

	brakeSolenoid_ = new Solenoid(BRAKE_SOLENOID_PORT);
	rampLegSolenoidL_ = new DoubleSolenoid(RAMP_LEG_SOLENOID_L_PORT_A, RAMP_LEG_SOLENOID_L_PORT_B);
	rampLegSolenoidR_ = new DoubleSolenoid(RAMP_LEG_SOLENOID_R_PORT_A, RAMP_LEG_SOLENOID_R_PORT_B);
	rampReleaseSolenoidL_ = new DoubleSolenoid(RAMP_RELEASE_SOLENOID_L_PORT_A, RAMP_RELEASE_SOLENOID_L_PORT_B);
	rampReleaseSolenoidR_ = new DoubleSolenoid(RAMP_RELEASE_SOLENOID_R_PORT_A, RAMP_RELEASE_SOLENOID_R_PORT_B);

	wristSolenoid_ = new DoubleSolenoid(WRIST_UP_SOLENOID_PORT, WRIST_DOWN_SOLENOID_PORT);

	intakeSensor_ = new DigitalInput(INTAKE_SENSOR_PWM_PORT);

	wristUp_ = true;

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

void RobotModel::SetTalonBrakeMode() {
	rightMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	leftMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	rightSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	leftSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void RobotModel::SetTalonCoastMode() {
	rightMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	leftMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	rightSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	leftSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void RobotModel::SetHighGear() {
	gearShiftSolenoid_->Set(DoubleSolenoid::kReverse); // TODO Check if right
}

void RobotModel::SetLowGear() {
	gearShiftSolenoid_->Set(DoubleSolenoid::kForward); // TODO Check if right
}

double RobotModel::GetLeftEncoderValue() {
	return leftDriveEncoder_->Get();
}

double RobotModel::GetRightEncoderValue() {
	return rightDriveEncoder_->Get();
}

double RobotModel::GetLeftDistance() {
	return -leftDriveEncoder_->GetDistance();
}

double RobotModel::GetRightDistance() {
	return -rightDriveEncoder_->GetDistance();
}

bool RobotModel::GetLeftEncoderStopped() {
	return leftDriveEncoder_->GetStopped();
}

bool RobotModel::GetRightEncoderStopped() {
	return rightDriveEncoder_->GetStopped();
}

void RobotModel::ResetDriveEncoders() {
	leftDriveEncoder_->Reset();
	rightDriveEncoder_->Reset();
}

double RobotModel::GetNavXYaw() {
	return navX_->GetYaw();
}

void RobotModel::ZeroNavXYaw() {
//	for (int i = 0; i < 4; i++) {
		navX_->ZeroYaw();
//	}
	printf("Zeroed Yaw\n");
}

double RobotModel::GetNavXPitch() {
	return navX_->GetPitch();
}

double RobotModel::GetNavXRoll() {
	return navX_->GetRoll();
}

void RobotModel::SetIntakeOutput(double output) {
	leftIntakeMotor_->Set(output);
	rightIntakeMotor_->Set(-output);
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

Encoder* RobotModel::GetElevatorEncoder() {
	return elevatorEncoder_;
}


void RobotModel::SetWristUp() {
	wristSolenoid_->Set(DoubleSolenoid::kForward); // TODO Check if right
	wristUp_ = true;
}

void RobotModel::ReleaseRampLegs() {
	rampLegSolenoidL_->Set(DoubleSolenoid::kForward);
	rampLegSolenoidR_->Set(DoubleSolenoid::kForward);
}

void RobotModel::ReleaseRamps() {
	rampReleaseSolenoidL_->Set(DoubleSolenoid::kForward);
	rampReleaseSolenoidR_->Set(DoubleSolenoid::kForward);
}

void RobotModel::SetRampMotorLOutput(double output) {
	rampLMotor_->Set(output);
}

void RobotModel::SetRampMotorROutput(double output) {
	rampRMotor_->Set(output);
}

void RobotModel::EngageBrake() {
	brakeSolenoid_->Set(false);
}

void RobotModel::DisengageBrake() {
	brakeSolenoid_->Set(true);
}

void RobotModel::SetWristDown() {
	wristSolenoid_->Set(DoubleSolenoid::kReverse); // TODO Check if right
	wristUp_ = false;
}

double RobotModel::GetElevatorCurrent() {
	return pdp_->GetCurrent(ELEVATOR_MOTOR_PDP_CHAN);
}

bool RobotModel::GetWristUp() {
	return wristUp_;
}

void RobotModel::StopCompressor() {
	compressor_->Stop();
}

void RobotModel::StartCompressor() {
	compressor_->Start();
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

	driveRPFac_ = pini_->getf("DRIVESTRAIGHT PID", "rPFac", 0.0);
	driveRIFac_ = pini_->getf("DRIVESTRAIGHT PID", "rIFac", 0.0);
	driveRDFac_ = pini_->getf("DRIVESTRAIGHT PID", "rDFac", 0.0);
	driveDPFac_ = pini_->getf("DRIVESTRAIGHT PID", "dPFac", 0.0);
	driveDIFac_ = pini_->getf("DRIVESTRAIGHT PID", "dIFac", 0.0);
	driveDDFac_ = pini_->getf("DRIVESTRAIGHT PID", "dDFac", 0.0);
	driveTimeoutSec_ = pini_->getf("DRIVESTRAIGHT PID", "driveTimeoutSec", 2.5);

	elevatorPFac_ = pini_->getf("ELEVATOR PID", "pFac", 0.0);
	elevatorIFac_ = pini_->getf("ELEVATOR PID", "iFac", 0.0);
	elevatorDFac_ = pini_->getf("ELEVATOR PID", "dFac", 0.0);

	cubeInSwitchL_ = pini_->gets("CUBE IN SWITCH", "cubeInSwitchL", "d10");
	cubeInSwitchR_ = pini_->gets("CUBE IN SWITCH", "cubeInSwitchR", "d10");

	intakeMotorOutput_ = pini_->getf("SUPERSTRUCTURE", "intakeMotorOutput", 0.0);
	outtakeMotorOutput_ = pini_->getf("SUPERSTRUCTURE", "outtakeMotorOutput", 0.0);

	testMode_ = pini_->gets("AUTO TEST", "sequence", "");
	//comment out autoPos if driver station is set up to test without ini file
	autoPos_ = pini_->getf("AUTO TEST", "pos", 0.0);
	autoMode_ = pini_->getf("AUTO TEST", "mode", 0.0);

}

void RobotModel::PrintState() {
	SmartDashboard::PutNumber("Left Drive Output", leftMaster_->Get());
	SmartDashboard::PutNumber("Right Drive Output", rightMaster_->Get());
	SmartDashboard::PutNumber("Left Drive Distance", GetLeftDistance());
	SmartDashboard::PutNumber("Right Drive Distance", GetRightDistance());
	SmartDashboard::PutNumber("Elevator Encoder Val", elevatorEncoder_->Get());
	SmartDashboard::PutNumber("NavX Yaw", GetNavXYaw());
	SmartDashboard::PutNumber("NavX Pitch", GetNavXPitch());
	SmartDashboard::PutNumber("NavX Roll", GetNavXRoll());
	SmartDashboard::PutNumber("Elevator Current", pdp_->GetCurrent(ELEVATOR_MOTOR_PDP_CHAN));
}

RobotModel::~RobotModel() {
	// TODO Auto-generated destructor stub
}

