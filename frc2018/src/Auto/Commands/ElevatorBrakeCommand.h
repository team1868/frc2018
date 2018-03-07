/*
 * ElevatorBrakeCommand.h
 *
 *  Created on: Mar 6, 2018
 *      Author: alisha
 */

#ifndef SRC_AUTO_COMMANDS_ELEVATORBRAKECOMMAND_H_
#define SRC_AUTO_COMMANDS_ELEVATORBRAKECOMMAND_H_

#include "Auto/Commands/AutoCommand.h"

class ElevatorBrakeCommand : public AutoCommand {
public:
	ElevatorBrakeCommand(RobotModel *robot, bool engageBrake);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	void Reset();
	virtual ~ElevatorBrakeCommand();
private:
	RobotModel *robot_;
	bool engageBrake_;
	bool isDone_;
};

#endif /* SRC_AUTO_COMMANDS_ELEVATORBRAKECOMMAND_H_ */
