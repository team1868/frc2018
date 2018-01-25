#ifndef SRC_AUTO_COMMANDS_DRIVESTRAIGHTCOMMAND_H_
#define SRC_AUTO_COMMANDS_DRIVESTRAIGHTCOMMAND_H_

#include "WPILib.h"
#include <math.h>
#include "RobotModel.h"
#include "Auto/Commands/AutoCommand.h"
#include "Auto/PIDSource/PIDInputSource.h"
#include "Auto/PIDSource/PIDOutputSource.h"

class DriveStraightCommand : public AutoCommand {
public:
	/**
	 * Constructor that generates the DriveStraight Command
	 * @param navXSource angle input for the PID Loop
	 * @param talonEncoderSource distance input for distance PID Loop
	 * @param anglePIDOutput output for the angle PID Loop
	 * @param robot robot model
	 * @param distance in feet
	 */
	DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
			AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
			double desiredDistance);
	/**
	 * Destructor
	 */

	virtual ~DriveStraightCommand();

	/**
	 * Chekcs if the PID is on target or if it times out, then sets isDone_ is true and stops the motors. Otherwise, sets the right and left motors to the PID outputs.
	 */

	void Init();

	void Update(double currTime, double deltaTime);

	/**
	 * @return isDone_
	 */

	bool IsDone();

	/**
	 * Disables the PIDControllers and make the command done.
	 */

	void Reset();

	/**
	 * Gets PID values from the ini file
	 */

	void GetIniValues();

private:
	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonEncoderSource_;
	AnglePIDOutput *anglePIDOutput_;
	DistancePIDOutput *distancePIDOutput_;
	PIDController *anglePID_;
	PIDController *distancePID_;
	RobotModel *robot_;

	double rPFac_, rIFac_, rDFac_;
	double rMaxOutput_, rTolerance_;
	double dPFac_, dIFac_, dDFac_;
	double dMaxOutput_, dTolerance_;
	double initialAngle_;
	double initialAvgDistance_;
	double desiredDistance_;
	double desiredTotalAvgDistance_;
	double leftMotorOutput_, rightMotorOutput_;
	double initialDriveTime_, diffDriveTime_;
	bool isDone_;
	int numTimesOnTarget_;

};

#endif /* SRC_AUTO_COMMANDS_DRIVESTRAIGHTCOMMAND_H_ */
