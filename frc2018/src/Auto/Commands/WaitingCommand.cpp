#include <Auto/Commands/WaitingCommand.h>

WaitingCommand::WaitingCommand(double myWaitTimeSec) : AutoCommand() {
	waitTimeSec_ = myWaitTimeSec;
	timer_ = new Timer();
	isDone_ = false;
}

void WaitingCommand::Init() {
	timer_->Start();
}

void WaitingCommand::Update(double currTimeSec, double deltaTimeSec) {
	isDone_ = (timer_->Get() >= waitTimeSec_);
	if(isDone_) {
		printf("done waiting %f", currTimeSec);
	}
}

bool WaitingCommand::IsDone() {
	return isDone_;
}

void WaitingCommand::Reset() {
}

WaitingCommand::~WaitingCommand() {
}
