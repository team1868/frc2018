#include <Auto/Modes/LeftSideToLeftSwitchMode.h>

LeftSideToLeftSwitchMode::LeftSideToLeftSwitchMode(RobotModel *robot) : AutoMode(robot) {
	printf("constructing left side to left switch mode\n");
	MPPathCommand_ = new PathCommand(robot_, PathCommand::kLeftSideToLeftSwitch, 341);
}

void LeftSideToLeftSwitchMode::CreateQueue(string gameData, AutoMode::AutoPositions pos) {
	printf("creating queue for left side to left switch mode\n");
	currentCommand_ = MPPathCommand_;
}

void LeftSideToLeftSwitchMode::Init() {
	currentCommand_->Init();
}

LeftSideToLeftSwitchMode::~LeftSideToLeftSwitchMode() {

}
