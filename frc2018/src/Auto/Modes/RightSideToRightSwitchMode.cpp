#include <Auto/Modes/RightSideToRightSwitchMode.h>

RightSideToRightSwitchMode::RightSideToRightSwitchMode() {
	printf("constructing right side to right switch mode\n");
	MPPathCommand_ = new PathCommand(robot_, PathCommand::kTestKOP, 341);
}

RightSideToRightSwitchMode::~RightSideToRightSwitchMode() {
	printf("creating queue for right side to right switch mode\n");
	currentCommand_ = MPPathCommand_;
}

void RightSideToRightSwitchMode::Init() {
	currentCommand_->Init();
}

RightSideToRightSwitchMode::~RightSideToRightSwitchMode() {

}

