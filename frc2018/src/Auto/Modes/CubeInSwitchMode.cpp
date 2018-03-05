/*
 * CubeInSwitchMode.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: starr
 */

#include <Auto/Modes/CubeInSwitchMode.h>

CubeInSwitchMode::CubeInSwitchMode(RobotModel *robot) : AutoMode(robot) {
	printf("In Cube In Switch Mode Constructor\n");
}

void CubeInSwitchMode::CreateQueue(string gameData, AutoMode::AutoPositions pos) {

	AutoPositions autoPos = pos;
	printf("autopositions: %d\n", pos);
	char switchSide = gameData[0];
	string autoSequence = "";

	// TODO fill in the auto sequence, add superstructure commands
	switch(autoPos) {
	case kLeft:
		printf("Left position: ");
		if (switchSide == 'L') {
			printf("Left switch: ");
			autoSequence = "d 12.0 t 90.0 d 1.81"; //tested on KOP
		} else if (switchSide == 'R') {
			printf("Right switch: ");
			autoSequence = "d 17.56 t 90.0 d 17.45 t 180.0 d 3.0 t -90.0"; // tested on KOP
		}
		break;
	case kMiddle:
		printf("Middle Position: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = "d 2.58 t -90.0 d 3.31 t 0.0 d 6.08"; // TODO outtake and test KOP
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = "d 2.583 t 90.0 d 3.02 t 0.0 d 6.08"; // TODO outtake and test on KOP
		}
		break;
	case kMiddleRight:
		printf("Middle Right: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = "d 8.67"; //TODO test
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = "d 8.67"; // TODO outtake, test
		}
		break;
	case kFarRight:
		printf("Far Right: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = "d 17.56 t -90.0 d 17.45 t -180.0 d 3.0 t 90.0"; // TODO outtake, test
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = "d 12.5 t -90.0 d 1.81"; // TODO outtake, test
		}
		break;
	case kIni:
		printf("Reading from Ini: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = robot_->cubeInSwitchL_;
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = robot_->cubeInSwitchR_;
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
	printf("In Cube In Switch Mode Queue\n");
}

void CubeInSwitchMode::Init() {
	printf("Cube In Switch Mode Init\n");
	currentCommand_->Init();
}

CubeInSwitchMode::~CubeInSwitchMode() {
	// TODO Auto-generated destructor stub
}

