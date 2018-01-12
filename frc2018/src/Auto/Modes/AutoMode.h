#ifndef SRC_AUTO_MODES_AUTOMODE_H_
#define SRC_AUTO_MODES_AUTOMODE_H_

#include "Auto/Commands/AutoCommand.h"

class AutoMode {
public:
	AutoMode() {
		currentCommand = NULL;
	};

	virtual ~AutoMode() {};

	virtual void CreateQueue() = 0;

	virtual void Init() = 0;

	/**
	 * Updates the current command and prints out if each command is complete and also
	 * if the automode update is finished
	 * @param currTimeSec a double current time
	 * @param deltaTimeSec a double how often to update
	 */
	void Update(double currTimeSec, double deltaTimeSec) {
		if (currentCommand != NULL) {
			if (currentCommand->IsDone()) {
//				DO_PERIODIC(1, printf("Command complete at: %f \n", currTimeSec));
				currentCommand = currentCommand->GetNextCommand();
				if (currentCommand != NULL) {
//					DO_PERIODIC(1, printf("Command start at: %f \n", currTimeSec));
					currentCommand->Init();
					printf("Initializing current commmand\n");
				}
			} else {
				currentCommand->Update(currTimeSec, deltaTimeSec);
			}
		} else {
			printf("Done with auto mode update\n");
		}
	}

	virtual void RefreshIni() = 0;

	/**
	 * Returns when AutoMode is done
	 * @return true if there is no current command or current command is NULL
	 */
	bool IsDone() {
		return (currentCommand == NULL);
	}

protected:
	AutoCommand *currentCommand;
};

#endif /* SRC_AUTO_MODES_AUTOMODE_H_ */
