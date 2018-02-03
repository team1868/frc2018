/*
 * IntakeCommand.h
 *
 *  Created on: Jan 30, 2018
 *      Author: alisha
 */

#ifndef SRC_AUTO_COMMANDS_INTAKECOMMAND_H_
#define SRC_AUTO_COMMANDS_INTAKECOMMAND_H_

#include "AutoCommand.h"
#include "RobotModel.h"

class IntakeCommand : public AutoCommand {
public:
	IntakeCommand(RobotModel *robot);
	virtual ~IntakeCommand();

	void Init();

	void Reset();

	void Update(double currTimeSec, double deltaTimeSec);

	bool IsDone();

private:

	bool isDone_;

	RobotModel *robot_;

	double intakeMotorOutput_;
};

#endif /* SRC_AUTO_COMMANDS_INTAKECOMMAND_H_ */
