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

	/**
	 * Sets intake/outake motors
	 * @param output
	 */
	void SetIntakeOutput(double output);

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
	void RefreshIni();

	/* ------------------ SUPERSTRUCTURE ------------------  */

	/* ------------------------------------------------------  */

	Ini *pini_;
	double pivotPFac_, pivotIFac_, pivotDFac_;
	double driveDPFac_, driveDIFac_, driveDDFac_;
	double driveRPFac_, driveRIFac_, driveRDFac_;
	double elevatorPFac_, elevatorIFac_, elevatorDFac_;
	double driveTimeoutSec_, pivotTimeoutSec_;
	string cubeInSwitchL_, cubeInSwitchR_;

	WPI_TalonSRX *leftMaster_, *rightMaster_;
	WPI_VictorSPX *leftSlave_, *rightSlave_;	// Fix Later

private:
	void RefreshIniVals();

	Timer *timer_;
	AHRS *navX_;

	Compressor *compressor_;
//	DoubleSolenoid *gearShiftSolenoid_;
	Encoder *leftDriveEncoder_, *rightDriveEncoder_;
	PowerDistributionPanel* pdp_;
	bool isLeftInverted_;

	Victor *leftIntakeMotor_, *rightIntakeMotor_, *elevatorMotor_;
	Encoder *elevatorEncoder_;
	DigitalInput *intakeSensor_;
};


#endif /* SRC_ROBOTMODEL_H_ */
