#ifndef AUTOCOMMAND_H
#define AUTOCOMMAND_H

#include "WPILib.h"
#include "Debugging.h"
#include "RobotModel.h"
#include <vector>
#include <string>
#include <iostream>

using namespace std;

class AutoCommand {
public:
	/**
	 * Constructor that generates a PathCommand
	 * If extended, allows other commands to implement these methods
	 */
	AutoCommand() {
		nextCommand_ = NULL;
	}

	virtual ~AutoCommand() {}

	virtual void Init() = 0;

	virtual void Update(double currTimeSec, double deltaTimeSec) = 0;

	virtual bool IsDone() = 0;

	virtual void Reset() = 0;

	AutoCommand* GetNextCommand() {
		return nextCommand_;
	}

	void SetNextCommand(AutoCommand* myNextCommand) {
		nextCommand_ = myNextCommand;
	}

private:
	AutoCommand *nextCommand_;
};
#endif /* AUTOCOMMAND_H */
