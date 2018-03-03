#include <Auto/Modes/KOPTestMPMode.h>

KOPTestMPMode::KOPTestMPMode(RobotModel *robot) : AutoMode(robot) {
	printf("Constructing KOP MP test\n");
//	MPPathCommand_ = new PathCommand(robot_, PathCommand::kTestKOP, 97);
	printf("hello\n");
}

void KOPTestMPMode::CreateQueue(string gameData, AutoMode::AutoPositions pos) {
	printf("creating queue for KOP MP test\n");
//	currentCommand_ = MPPathCommand_;
}

void KOPTestMPMode::Init() {
	currentCommand_->Init();
}

KOPTestMPMode::~KOPTestMPMode() {

}
