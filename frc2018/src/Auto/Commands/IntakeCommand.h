/*
 * IntakeCommand.h
 *
 *  Created on: Jan 30, 2018
 *      Author: alisha
 */

#ifndef SRC_AUTO_COMMANDS_INTAKECOMMAND_H_
#define SRC_AUTO_COMMANDS_INTAKECOMMAND_H_

#include "Auto/Commands/AutoCommand.h"
#include "RobotModel.h"

class IntakeCommand : public AutoCommand {
public:
	IntakeCommand(RobotModel *robot, double intakeMotorOutput = 0.7); //TODO TEST
	virtual ~IntakeCommand();

	void Init();

	void Reset();

	void Update(double currTimeSec, double deltaTimeSec);

	bool IsDone();

private:

	bool isDone_;

	RobotModel *robot_;

	double intakeMotorOutput_;
	double startTime_;
	double timeDiff_;
	bool wasJustRunning_;
};

#endif /* SRC_AUTO_COMMANDS_INTAKECOMMAND_H_ */
