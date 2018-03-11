/*
 * ParallelCommand.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: starr
 */

#include <Auto/Commands/ParallelCommand.h>

ParallelCommand::ParallelCommand(AutoCommand *commandA, AutoCommand *commandB) : AutoCommand() {
	printf("Constructing parallel auto command\n");
	commandA_ = commandA;
	commandB_ = commandB;
}

void ParallelCommand::Init() {
	printf("Initializing parallel auto command\n");
	commandA_->Init();
	commandB_->Init();
}

void ParallelCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (!IsDone()) {
		if (commandA_->IsDone()) {	// Keep commandB running if commandA is done
			commandA_->Reset();
			commandB_->Update(currTimeSec, deltaTimeSec);
		} else if (commandB_->IsDone()) {	// Keep commandA running if commandB is done
			commandB_->Reset();
			commandA_->Update(currTimeSec, deltaTimeSec);
		} else {	// If both aren't done, keep both commands updating
			commandA_->Update(currTimeSec, deltaTimeSec);
			commandB_->Update(currTimeSec, deltaTimeSec);
		}
	}
}

void ParallelCommand::Reset() {
	commandA_->Reset();
	commandB_->Reset();
}

bool ParallelCommand::IsDone() {
	return (commandA_->IsDone() && commandB_->IsDone());
}

ParallelCommand::~ParallelCommand() {
	// TODO Auto-generated destructor stub
}

