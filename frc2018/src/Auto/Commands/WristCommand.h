/*
 * WristCommand.h
 *
 *  Created on: Mar 4, 2018
 *      Author: starr
 */

#include "Auto/Commands/AutoCommand.h"

#ifndef SRC_AUTO_COMMANDS_WRISTCOMMAND_H_
#define SRC_AUTO_COMMANDS_WRISTCOMMAND_H_

class WristCommand : public AutoCommand {
public:
	WristCommand(RobotModel *robot, bool wristUp);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	void Reset();
	virtual ~WristCommand();
private:
	RobotModel *robot_;
	bool wristUp_;
	bool isDone_;
};

#endif /* SRC_AUTO_COMMANDS_WRISTCOMMAND_H_ */
