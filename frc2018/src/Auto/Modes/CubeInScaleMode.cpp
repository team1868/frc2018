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
			autoSequence = "d 21.2 e 5.4";
					//"p d 21.25 e 5.4 t 70.0 t 70.0 d 1.5 o"; // old stuffz: p d 25.971 e 5.4 t 90.0 d 1.50 o
		} else if (scaleSide == 'R') {
			printf("Right scale: ");
			autoSequence = "d 17.5 t 90.0 t 90.0 d 17.0";
					//"d 17.5 t 90.0 t 90.0 p e 5.4 d 17.0 t -10.0 t -10.0 d 3.0 o"; // TODO test, angles should be in absolute
		}
		break;
	case kFarRight:
		printf("Far Right: ");
		if (scaleSide == 'L') {
			printf("Left Scale: ");
			autoSequence = "d 17.5 t -90 t -90.0 p e 5.4 d 17.0 t 10.0 t 10.0 d 3.0 o"; // TODO test, angles should be in absolute
		} else if (scaleSide == 'R') {
			printf("Right Scale: ");
			autoSequence = "p d 21.25 e 5.4 t -70.0 t -70.0 d 1.50 o"; // TODO test
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
	QueueFromString(autoSequence);	// TODO uncomment this when ready
	printf("In Cube In Scale Mode Queue\n");
}

void CubeInScaleMode::Init() {
	printf("Cube In Scale Mode Init\n");
	currentCommand_->Init();
}

CubeInScaleMode::~CubeInScaleMode() {
	// TODO Auto-generated destructor stub
}

