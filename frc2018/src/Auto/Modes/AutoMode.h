#ifndef SRC_AUTO_MODES_AUTOMODE_H_
#define SRC_AUTO_MODES_AUTOMODE_H_

#include "Auto/Commands/AutoCommand.h"

class AutoMode {
public:
	AutoMode() {
		currentCommand_ = NULL;
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
		if (currentCommand_ != NULL) {
			if (currentCommand_->IsDone()) {
//				DO_PERIODIC(1, printf("Command complete at: %f \n", currTimeSec));
				currentCommand_ = currentCommand_->GetNextCommand();
				if (currentCommand_ != NULL) {
//					DO_PERIODIC(1, printf("Command start at: %f \n", currTimeSec));
					currentCommand_->Init();
					printf("Initializing current commmand\n");
				}
			} else {
				currentCommand_->Update(currTimeSec, deltaTimeSec);
			}
		} else {
//			printf("Done with auto mode update\n");
		}
	}

	virtual void RefreshIni() = 0;

	/**
	 * Returns when AutoMode is done
	 * @return true if there is no current command or current command is NULL
	 */
	bool IsDone() {
		return (currentCommand_ == NULL);
	}

	void Disable(){
		printf("Disabling\n");
		if (!IsDone()) {
			currentCommand_->Reset();
		}
		printf("Successfully disabled\n");
	}

protected:
	AutoCommand *currentCommand_;
};

#endif /* SRC_AUTO_MODES_AUTOMODE_H_ */
