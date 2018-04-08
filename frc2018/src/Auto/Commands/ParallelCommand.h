/*
 * ParallelCommand.h
 *
 *  Created on: Feb 19, 2018
 *      Author: starr
 */

#ifndef SRC_AUTO_COMMANDS_PARALLELCOMMAND_H_
#define SRC_AUTO_COMMANDS_PARALLELCOMMAND_H_

#include "Auto/Commands/AutoCommand.h"

class ParallelCommand : public AutoCommand{
public:
	ParallelCommand(AutoCommand *commandA, AutoCommand *commandB);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	void Reset();
	virtual ~ParallelCommand();

	enum AutoStates {
		kBoth, kCommandA, kCommandB, kIsDone
	};

private:
	uint32_t currState_;
	uint32_t nextState_;

	AutoCommand *commandA_;
	AutoCommand *commandB_;

	AutoCommand *firstCommandA_;
	AutoCommand *firstCommandB_;
};

#endif /* SRC_AUTO_COMMANDS_PARALLELCOMMAND_H_ */
