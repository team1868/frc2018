#include <Auto/Modes/LeftSideToLeftSwitchMode.h>

LeftSideToLeftSwitchMode::LeftSideToLeftSwitchMode() {
	printf("constructing left side to left switch mode\n");
	MPPathCommand_ = new PathCommand(robot_, PathCommand::kTestKOP, 341);
}

LeftSideToLeftSwitchMode::~LeftSideToLeftSwitchMode() {
	printf("creating queue for left side to left switch mode\n");
	currentCommand_ = MPPathCommand_;
}

void LeftSideToLeftSwitchMode::Init() {
	currentCommand_->Init();
}

LeftSideToLeftSwitchMode::~LeftSideToLeftSwitchMode() {

}
