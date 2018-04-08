/*
 * DriveToCubeCommand.h
 *
 *  Created on: Apr 8, 2018
 *      Author: starr
 */

#include "Auto/Commands/AutoCommand.h"
#include "Auto/Commands/DriveIntakeCubeCommand.h"
#include "Auto/Commands/PivotToCubeCommand.h"

#ifndef SRC_AUTO_COMMANDS_DRIVETOCUBECOMMAND_H_
#define SRC_AUTO_COMMANDS_DRIVETOCUBECOMMAND_H_

using namespace std;

class DriveToCubeCommand : public AutoCommand{
public:
	DriveToCubeCommand(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource, AnglePIDOutput *angleOutput, DistancePIDOutput *distanceOutput);
	virtual ~DriveToCubeCommand();

	void Init();
	void Reset();
	void Update(double currTimeSEc, double deltaTimeSec);
	bool IsDone();

	void ReadFromJetson();
	std::string ReceiveMsgNoBlock(zmq::socket_t & socket);

private:
	RobotModel *robot_;

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonSource_;
	AnglePIDOutput * angleOutput_;
	DistancePIDOutput *distanceOutput_;

	PivotToCubeCommand* driveToCubeCommand_;
	DriveIntakeCubeCommand* intakingCommand_;
	AutoCommand* currentCommand_;

	bool isDone_;
};

#endif /* SRC_AUTO_COMMANDS_DRIVETOCUBECOMMAND_H_ */
