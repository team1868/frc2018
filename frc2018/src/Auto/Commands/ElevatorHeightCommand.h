/*
 * ElevatorHeightCommand.h
 *
 *  Created on: Jan 29, 2018
 *      Author: alisha
 */

#ifndef SRC_AUTO_COMMANDS_ELEVATORHEIGHTCOMMAND_H_
#define SRC_AUTO_COMMANDS_ELEVATORHEIGHTCOMMAND_H_

#include "WPILib.h"
#include "RobotModel.h"
#include "Auto/Commands/AutoCommand.h"
#include "Auto/PIDSource/PIDOutputSource.h"

class ElevatorHeightCommand : public AutoCommand{
public:
	ElevatorHeightCommand(RobotModel *robot, double desiredHeight);
	ElevatorHeightCommand(RobotModel *robot);

	virtual ~ElevatorHeightCommand();

	void Init();

	void Reset();

	void Update(double currTimeSec, double deltaTimeSec);

	void SetSetpoint(double newHeight);

	bool IsDone();

	void GetIniValues();

private:
	double pFac_, iFac_, dFac_;

	double desiredHeight_;

	bool isDone_;
	int numTimesOnTarget_;

	RobotModel *robot_;
	PIDController *elevatorHeightPID_;

	ElevatorEncoderPIDSource *encoderPIDSource_;
	Victor *elevatorPIDOutput_;

	double maxOutput_;
	double tolerance_;

	double startTime_;
};

#endif /* SRC_AUTO_COMMANDS_ELEVATORHEIGHTCOMMAND_H_ */
