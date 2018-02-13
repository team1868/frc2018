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
			autoSequence = "d 12.0 p 90.0 d 1.81 o";
		} else if (switchSide == 'R') {
			printf("Right switch: ");
			autoSequence = "d 17.56 p 90.0 d 17.6 p -118.81 d 1.61 o";
		}
		break;
	case kMiddle:
		printf("Middle Position: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = "d 2.58 p -90.0 d 3.31 p 0.0 d 6.08 o";
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = "d 2.583 p 90.0 d 3.02 p 0.0 d 6.08 o";
		}
		break;
	case kMiddleRight:
		printf("Middle Right: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = "d 8.67";
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = "d 8.67 o";
		}
		break;
	case kFarRight:
		printf("Far Right: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = "d 17.56 p -90 d 17.6 p 118.81 d 1.61 o";
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = "d 12.5 p -90.0 d 1.81 o";
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
	//	QueueFromString(autoSequence);	// TODO uncomment this when ready
	printf("In Cube In Switch Mode Queue\n");
}

void CubeInSwitchMode::Init() {
	printf("Cube In Switch Mode Init\n");
	currentCommand_->Init();
}

CubeInSwitchMode::~CubeInSwitchMode() {
	// TODO Auto-generated destructor stub
}

