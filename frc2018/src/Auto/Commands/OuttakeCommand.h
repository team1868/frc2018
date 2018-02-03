/*
 * OuttakeCommand.h
 *
 *  Created on: Jan 31, 2018
 *      Author: alisha
 */

#ifndef SRC_AUTO_COMMANDS_OUTTAKECOMMAND_H_
#define SRC_AUTO_COMMANDS_OUTTAKECOMMAND_H_

#include "RobotModel.h"

class OuttakeCommand {
public:
	OuttakeCommand(RobotModel* robot);
	virtual ~OuttakeCommand();

	void Init();
	void Reset();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	bool isDone_;
	RobotModel *robot_;
	double outtakeMotorOutput_;
	double startTime_;
	double deltaTime_;

};

#endif /* SRC_AUTO_COMMANDS_OUTTAKECOMMAND_H_ */
