#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include <math.h>
#include <fstream>
#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include "Ports2018.h"
#include "../ext/ini/ini.h"

using namespace std;

class RobotModel {
public:
	enum Wheels { kLeftWheels, kRightWheels, kAllWheels };
	enum Talons { kLeftMaster, kRightMaster};

	/**
	 * Creates all objects regarding the robot parts. Configures the left and right
	 * talons. Initializes all variables
	 */
	RobotModel();

	/**
	 * Destructor
	 */
	~RobotModel();

	/**
	 * Resets timer
	 */
	void ResetTimer();

	/**
	 * @returns timer get
	 */
	double GetTime();

	/* -------------------- DRIVE --------------------  */
	WPI_TalonSRX *GetTalon(Talons talon);

	/**
	 * Sets specified side of talons to a value
	 * @param wheel the side of talons (left, right, all)
	 * @param value the value to set the talon motors to (-1, 1)
	 */
	void SetDriveValues(Wheels wheel, double value);

	void SetTalonBrakeMode();

	void SetTalonCoastMode();

	/**
	 * Sets drive to high gear
	 */
	void SetHighGear();

	/**
	 * Sets drive to low gear
	 */
	void SetLowGear();

	/**
	 * @return native left encoder value
	 */
	double GetLeftEncoderValue();

	/**
	 * @return native right encoder value
	 */
	double GetRightEncoderValue();

	/**
	 * @return left wheel distance
	 */
	double GetLeftDistance();
	/**
	 * @return right wheel distance
	 */
	double GetRightDistance();

	/**
	 * @return if left encoder isn't turning
	 */
	bool GetLeftEncoderStopped();

	/**
	 * @return if right encod
	 */
	bool GetRightEncoderStopped();

	void ResetDriveEncoders();

	/**
	 * @return angle from navX
	 */
	double GetNavXYaw();

	/**
	 * Zeroes the navX
	 */
	void ZeroNavXYaw();

	/**
	 * @return pitch from navX
	 */
	double GetNavXPitch();

	/**
	 * @return roll from navX
	 */
	double GetNavXRoll();

	/* ------------------ SUPERSTRUCTURE ------------------  */

	/**
	 * Sets intake/outake motors
	 * @param output
	 */
	void SetIntakeOutput(double output);
	void SetIntakeOutput(double leftOutput, double rightOutput);

	void SetWristOutput(double output);

	/**
	 * Sets elevator output
	 * @param output
	 */
	void SetElevatorOutput(double output);

	/**
	 * @return whether the cube is in the intake or not
	 */
	bool GetCubeInIntake();

	/**
	 * @return the elevator cube height
	 */
	double GetElevatorHeight();

	/**
	 * @return elevator victor
	 */
	Victor* GetElevatorMotor();

	/**
	 * Deletes the ini object and creates it again. Sets the ini values to whatever it reads
	 */

	Encoder* GetElevatorEncoder();

	double GetIntakeMotorSpeed();

	double GetOuttakeMotorSpeed();

	double GetWheelSpeed(RobotModel::Wheels wheel);

	void SetWristUp();

	void SetWristDown();

	bool GetWristUp();

	void ReleaseRampLegs();

	void ReleaseRamps();

	void SetRampMotorLOutput(double output);

	void SetRampMotorROutput(double output);

	void EngageBrake();

	void DisengageBrake();

	double GetElevatorCurrent();

	double GetWristCurrent();

	void StopCompressor();

	void StartCompressor();

	double GetPressureSensorVal(); //returns the pressure

	void RefreshIni();

	void PrintState();

	bool CollisionDetected();

	void UpdateCurrent(); //initializes variables pertaining to current

	double GetVoltage(); //returns the voltage

	double GetTotalEnergy(); //returns the total energy of the PDP

	double GetTotalCurrent(); //returns the total current of the PDP

	double GetTotalPower(); //returns the total power of the PDP

	double GetCurrent(int channel); //returns the current of a given channel

	double GetCompressorCurrent(); //returns the current of the compressor

	double GetRIOCurrent(); //returns the current of the roboRIO



	//double GetVoltage();


	/* ------------------------------------------------------  */

	Ini *pini_;
	double pivotPFac_, pivotIFac_, pivotDFac_;
	double driveDPFac_, driveDIFac_, driveDDFac_;
	double driveRPFac_, driveRIFac_, driveRDFac_;
	double elevatorPFac_, elevatorIFac_, elevatorDFac_, elevatorMaxOutput_, elevatorTallMaxOutput_, elevatorRampRate_;
	double driveTimeoutSec_, pivotTimeoutSec_;

	double elevatorOutput_;
	double leftDriveOutput_, rightDriveOutput_;
	double intakeMotorOutput_, outtakeMotorOutput_, outtakeFastMotorOutput_, intakeMotorOutputSubtract_;
	double wristMotorOutput_;
	double wristPFac_;
	double driveCurrentLimit_, intakeCurrentLimit_, totalCurrentLimit_, voltageFloor_, pressureFloor_, size_;

	double leftDriveACurrent_, leftDriveBCurrent_, rightDriveACurrent_, rightDriveBCurrent_,
			roboRIOCurrent_, compressorCurrent_, leftIntakeCurrent_, rightIntakeCurrent_;

	double last_world_linear_accel_x_;
	double last_world_linear_accel_y_;

	int autoPos_, autoMode_;
	string cubeInSwitchL_, cubeInSwitchR_;
	string testMode_;

	WPI_TalonSRX *leftMaster_, *rightMaster_;
	WPI_VictorSPX *leftSlave_, *rightSlave_;	// Fix Later



private:
	void RefreshIniVals();

	Timer *timer_;
	AHRS *navX_;
	int navXSpeed_; // In Hz

	Compressor *compressor_;
	DoubleSolenoid *gearShiftSolenoid_;
	Encoder *leftDriveEncoder_, *rightDriveEncoder_;
	PowerDistributionPanel* pdp_;
	bool isLeftInverted_;

	Victor *leftIntakeMotor_, *rightIntakeMotor_, *elevatorMotor_,  *rampLMotor_, *rampRMotor_;
	Victor *wristMotor_;
	Encoder *elevatorEncoder_;
	DoubleSolenoid *wristSolenoid_, *rampLegSolenoidL_, *rampLegSolenoidR_, *rampReleaseSolenoidL_, *rampReleaseSolenoidR_;
	AnalogInput *pressureSensor_; //TODO add
	DigitalInput *intakeSensor_;
	bool wristUp_;
	AnalogPotentiometer *wristPot_;
};


#endif /* SRC_ROBOTMODEL_H_ */
