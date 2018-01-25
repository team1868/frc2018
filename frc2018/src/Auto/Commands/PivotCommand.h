#ifndef SRC_AUTO_COMMANDS_PIVOTCOMMAND_H_
#define SRC_AUTO_COMMANDS_PIVOTCOMMAND_H_

#include "WPILib.h"
#include <AHRS.h>
#include "AutoCommand.h"
#include "Auto/PIDSource/PIDInputSource.h"
#include "Auto/PIDSource/PIDOutputSource.h"
#include "RobotModel.h"
#include "Profiler.h"

/**
 * PivotPIDTalonOutput a constructor Class for WPI PIDOutput for Pivoting
 */
class PivotPIDTalonOutput : public frc::PIDOutput {

public:

	/**
	 * PivotPIDTalonOutput is a constructor that initializes robot_ and output_
	 * @param robot a RobotModel
	 */
	PivotPIDTalonOutput(RobotModel *robot);

	/**
	 * PIDWrite a function that initializes output_ and sets the drive values
	 */
	void PIDWrite(double output);

	/**
	 * PivotPIDTalonOutput is a destructor
	 */
	virtual ~PivotPIDTalonOutput();

	/**
	 * @return output
	 */
	double GetOutput();

private:
	RobotModel *robot_;
	double output_;
};

/**
 * A class implementing Pivot PID the WPILibrary PID Controller
 */
class PivotCommand : public AutoCommand {

public:
	/**
	 * PivotCommand a constructor
	 * @param robot a RobotModel
	 * @param desiredAngle a double that is the angle of the turn
	 * @param isAbsolutePosition a bool that represents whether the angle is absolute position of deta angle
	 * @param navXSource a NavXPIDSource
	 */
	PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavXPIDSource* navXSource);

	/**
	 * PivotCommand a destructor
	 */
	virtual ~PivotCommand();

	/**
	 * gets Yaw from navX, sets Setpoint, continuous to false, output range, and absolute tolerance
	 */
	void Init();

	/**
	 * resets PID, sets isDone_ to true
	 */
	void Reset();

	/**
	 * if PivotPID is on target more than three times then timeout
	 * pivotPID resets, disable, isDone sets to true
	 */
	void Update(double currTimeSec, double deltaTimeSec);

	/**
	 * @return isDone_
	 */
	bool IsDone();

	/**
	 * gets PID values from ini file, sets to 0 if not present
	 */
	void GetIniValues();

private:
	/**
	 * calculates the angle needed to get to the desired angle
	 * @param desiredAngle a double that is the angle of the turn
	 */
	double CalculateDeltaAngle(double desiredAngle);

	double pFac_, iFac_, dFac_;
	double desiredAngle_;
	double initYaw_;

	bool isDone_;

	int numTimesOnTarget_;

	RobotModel *robot_;
	PIDController *pivotPID_;
	NavXPIDSource * navXSource_;
	PivotPIDTalonOutput *talonOutput_;

	double maxOutput_;
	double tolerance_;

	/**
	 * Minimum output to correct for, less would be considered done
	 */
	double minDrivePivotOutput_;

	double pivotCommandStartTime_;
};

#endif /* SRC_AUTO_COMMANDS_PIVOTCOMMAND_H_ */
