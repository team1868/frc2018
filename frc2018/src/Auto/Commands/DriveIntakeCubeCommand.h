/*
 * DriveIntakeCubeCommand.h
 *
 *  Created on: Apr 7, 2018
 *      Author: starr
 */
#include "Auto/Commands/AutoCommand.h"
#include "Auto/Commands/DriveStraightCommand.h"
#include "Auto/Commands/ElevatorHeightCommand.h"
#include "Auto/Commands/IntakeCommand.h"
#include "Auto/Commands/ParallelCommand.h"
#include "Auto/Commands/WristCommand.h"

#ifndef SRC_AUTO_COMMANDS_DRIVEINTAKECUBECOMMAND_H_
#define SRC_AUTO_COMMANDS_DRIVEINTAKECUBECOMMAND_H_

/**
 * This class just drive straight towards the cube while intaking it. Then it drives back, raising the elevator up, and
 * finally securing the cube with the wrist up.
 * Main use for two cube auto
 */
class DriveIntakeCubeCommand : public AutoCommand {
public:
	DriveIntakeCubeCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
			AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot);
	virtual ~DriveIntakeCubeCommand();

	void Init();

	void Reset();

	void Update(double currTimeSec, double deltaTimeSec);

	bool IsDone();

private:
	DriveStraightCommand* forwardCommand_;
	DriveStraightCommand* backwardCommand_;
	IntakeCommand* intakeFirst_;
	IntakeCommand* intakeLast_;
	ElevatorHeightCommand* elevatorCommand_;
	WristCommand* wristUp_;
	ParallelCommand* parallel1A_;
	ParallelCommand* parallel2A_;
	ParallelCommand* parallel2B_;

	AutoCommand* currentCommand_;
	AutoCommand* firstCommand_;

	RobotModel* robot_;
	bool isDone_;
};

#endif /* SRC_AUTO_COMMANDS_DRIVEINTAKECUBECOMMAND_H_ */
