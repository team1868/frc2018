/*
 * ParallelCommand.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: starr
 */

#include <Auto/Commands/ParallelCommand.h>

ParallelCommand::ParallelCommand(AutoCommand *commandA, AutoCommand *commandB) : AutoCommand() {
	printf("Constructing parallel auto command\n");
	currState_ = kBoth;
	nextState_ = currState_;

	firstCommandA_ = commandA;
	firstCommandB_ = commandB;

	commandA_ = firstCommandA_;
	commandB_ = firstCommandB_;
}

void ParallelCommand::Init() {
	printf("Initializing parallel auto command\n");
	commandA_->Init();
	commandB_->Init();

	currState_ = kBoth;
	nextState_ = currState_;
}

void ParallelCommand::Update(double currTimeSec, double deltaTimeSec) {
//	printf("In Update\n");
	switch(currState_) {
	case kBoth:
//		printf("In kBoth\n");
		if (!IsDone()) {
			commandA_->Update(currTimeSec, deltaTimeSec);
			commandB_->Update(currTimeSec, deltaTimeSec);

			if(commandA_->IsDone()) {
				commandA_ = commandA_->GetNextCommand();
				if (commandA_ == NULL) {
					nextState_ = kCommandB;
				} else {
					commandA_ ->Init();
					nextState_ = kBoth;
				}
			}
			if(commandB_->IsDone()) {
				commandB_ = commandB_->GetNextCommand();
				if (commandB_ == NULL) {
					if (commandA_ != NULL) {
						nextState_ = kCommandA;
					} else {
						Reset();
						nextState_ = kIsDone;
					}
				} else {
					commandB_->Init();
					nextState_ = kBoth;
				}
			}
		} else {
			printf("IS DONE FROM PARALLEL\n");
			Reset();
			nextState_ = kIsDone;
		}
		break;

	case kCommandA:
//		printf("In kCommandA\n");
		if (commandA_ != NULL) {
			commandA_->Update(currTimeSec, deltaTimeSec);
			if(commandA_->IsDone()) {
				commandA_ = commandA_->GetNextCommand();
				if (commandA_ == NULL) {
					Reset();
					nextState_ = kIsDone;
				} else {
					commandA_->Init();
					nextState_ = kCommandA;
				}
			}
		} else {
			printf("IS DONE FROM COMMANDA\n");
			Reset();
			nextState_ = kIsDone;
		}
		break;

	case kCommandB:
//		printf("In kCommandB\n");
		if (commandB_ != NULL) {
			commandB_->Update(currTimeSec, deltaTimeSec);
			if(commandB_->IsDone()) {
				commandB_ = commandB_->GetNextCommand();
				if (commandB_ == NULL) {
					Reset();
					nextState_ = kIsDone;
				} else {
					commandB_->Init();
					nextState_ = kCommandB;
				}
			}
		} else {
			printf("IS DONE FROM COMMANDB\n");
			Reset();
			nextState_ = kIsDone;
		}
		break;
	case kIsDone:
		printf("Parallel Command is done\n");
		nextState_ = kIsDone;
		break;

	}
	currState_ = nextState_;
}

void ParallelCommand::Reset() {
	printf("Resetting from parallel command\n");
	if (!IsDone()) {
		printf("Resetting current command\n");
		if (commandA_ != NULL) {
			commandA_->Reset();
		}
		if (commandB_ != NULL) {
			commandB_->Reset();
		}
	}

	if (firstCommandA_ != NULL) {
		printf("Deleting commandA \n");
		commandA_ = firstCommandA_;
		AutoCommand* nextCommand;
		while (commandA_ != NULL) {
			nextCommand = commandA_->GetNextCommand();
			delete(commandA_);
			commandA_ = NULL;
			commandA_ = nextCommand;
		}
		firstCommandA_ = NULL;
	}

	if (firstCommandB_ != NULL) {
		printf("Deleting commandB \n");
		commandB_ = firstCommandB_;
		AutoCommand* nextCommand;
		while (commandB_ != NULL) {
			nextCommand = commandB_->GetNextCommand();
			delete(commandB_);
			commandB_ = NULL;
			commandB_ = nextCommand;
		}
		firstCommandB_ = NULL;
	}
	printf("Successfully finished resetting parallel command\n");
}

bool ParallelCommand::IsDone() {
	return (commandA_ == NULL && commandB_ == NULL);
}

ParallelCommand::~ParallelCommand() {
	// TODO Auto-generated destructor stub
}

