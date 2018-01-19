#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include <math.h>
#include <fstream>
#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include "Ports2018.h"
#include "../ext/ini/ini.h"
#include "../ext/pathfinder/pathfinder.h"

using namespace std;

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

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
	 * @param wheel the side of the talon
	 * @return native encoder value
	 */
	double GetDriveEncoderValue(Wheels wheel);

	/**
	 * @return left wheel distance
	 */
	double GetLeftDistance();
	/**
	 * @return right wheel distance
	 */
	double GetRightDistance();

	/**
	 * Returns angle from navX
	 */
	double GetNavXYaw();

	/**
	 * Zeroes the navX
	 */
	void ZeroNavXYaw();
	void RefreshIni();

	/* ------------------ SUPERSTRUCTURE ------------------  */

	/* ------------------------------------------------------  */

	Ini *pini_;

	WPI_TalonSRX *leftMaster_, *rightMaster_, *leftSlave_, *rightSlave_;	// Fix Later

private:
	Timer *timer_;
	AHRS *navX_;

	Compressor *compressor_;
	DoubleSolenoid *gearShiftSolenoid_;
	Encoder *leftDriveEncoder_, *rightDriveEncoder_;
	PowerDistributionPanel* pdp_;
	bool isLeftInverted_;
};


#endif /* SRC_ROBOTMODEL_H_ */
