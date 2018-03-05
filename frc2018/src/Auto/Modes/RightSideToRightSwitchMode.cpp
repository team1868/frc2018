#include <Auto/Modes/RightSideToRightSwitchMode.h>

RightSideToRightSwitchMode::RightSideToRightSwitchMode(RobotModel *robot) : AutoMode(robot) {
	printf("constructing right side to right switch mode\n");
	MPPathCommand_ = new PathCommand(robot_, PathCommand::kRightSideToRightSwitch, 341);
}

void RightSideToRightSwitchMode::CreateQueue(string gameData, AutoMode::AutoPositions pos) {
	printf("creating queue for right side to right switch mode\n");
	currentCommand_ = MPPathCommand_;
}

void RightSideToRightSwitchMode::Init() {
	currentCommand_->Init();
}

RightSideToRightSwitchMode::~RightSideToRightSwitchMode() {

}

