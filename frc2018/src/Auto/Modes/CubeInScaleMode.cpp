/**
 * Created on March 3, 2018
 * Graceyraspberry
 */

#include <Auto/Modes/CubeInScaleMode.h>

CubeInScaleMode::CubeInScaleMode(RobotModel *robot) : AutoMode(robot) {
	printf("In Cube In Switch Mode Constructor\n");
}

void CubeInScaleMode::CreateQueue(string gameData, AutoMode::AutoPositions pos) {

	AutoPositions autoPos = pos;
	printf("autopositions: %d\n", pos);
	char scaleSide = gameData[1];
	string autoSequence = "";

	// TODO fill in the auto sequence, add superstructure commands
	switch(autoPos) {
	case kLeft:
		printf("Left position: ");
		if (scaleSide == 'L') {
			printf("Left scale: ");
			autoSequence = "d 21.273 t 45.0 d 1.61"; // TODO outtake
		} else if (scaleSide == 'R') {
			printf("Right scale: ");
			autoSequence = "d 228.735 t 90 d 228.13 t -45 d 46.294"; // TODO outtake
		}
		break;
	case kFarRight:
		printf("Far Right: ");
		if (scaleSide == 'L') {
			printf("Left Scale: ");
			autoSequence = "d 228.735 t -90 d 228.13 t 45 d 46.294"; // TODO outtake
		} else if (scaleSide == 'R') {
			printf("Right Scale: ");
			autoSequence = "d 21.273 t -45.0 d 1.61"; // TODO outtake
		}
		break;
	case kIni:
		printf("Reading from Ini: ");
		if (scaleSide == 'L') {
			printf("Left Switch: ");
			autoSequence = robot_->cubeInSwitchL_; //TODO add scale to Ini
		} else if (scaleSide == 'R') {
			printf("Right Switch: ");
			autoSequence = robot_->cubeInSwitchR_; //TODO add scale to Ini
		}
		break;
	case kBlank:
		autoSequence = "";
		printf("blank\n");
		break;
	default:
		autoSequence = "";
		printf("default\n");
		break;
	}

	printf("%s\n", autoSequence.c_str());
	//	QueueFromString(autoSequence);	// TODO uncomment this when ready
	printf("In Cube In Scale Mode Queue\n");
}

void CubeInScaleMode::Init() {
	printf("Cube In Scale Mode Init\n");
//	currentCommand_->Init();
}

CubeInScaleMode::~CubeInScaleMode() {
	// TODO Auto-generated destructor stub
}

