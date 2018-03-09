#ifndef SRC_AUTO_COMMANDS_WAITINGCOMMAND_H_
#define SRC_AUTO_COMMANDS_WAITINGCOMMAND_H_

#include "Auto/Commands/AutoCommand.h"
#include "RobotModel.h"

class WaitingCommand: public AutoCommand {
public:
	/**
	 * Assigns the waitTimeSec and creates the timer
	 */
	WaitingCommand(double myWaitTimeSec);

	/**
	 * Destructor
	 */
	virtual ~WaitingCommand();

	/**
	 * Starts the timer
	 */
	void Init();

	/**
	 * Checks if the timer meets the waitTimeSec. If so, isDone is set to true.
	 */
	void Update(double currTimeSec, double deltaTimeSec);

	/**
	 * @return isDone
	 */
	bool IsDone();

	void Reset();
private:
	double waitTimeSec_;
	Timer *timer_;
	bool isDone_;
};

#endif /* SRC_AUTO_COMMANDS_WAITINGCOMMAND_H_ */
