/*
 * CubeInSwitchMode.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: Lynn D
 */

#include <Auto/Modes/CubeInSwitchMode.h>

CubeInSwitchMode::CubeInSwitchMode(RobotModel *robot, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoder) : AutoMode(robot, navX, talonEncoder) {
	printf("In Cube in Switch Mode Constructor\n");
}

void CubeInSwitchMode::CreateQueue(string gameData, AutoPositions pos) {
	printf("In Cube switch mode queue\n");

	AutoPositions autoPos = pos;
	printf("autopositions: %d\n", pos);
	char switchSide = gameData[0];
	string autoSequence = "";

	// TODO fill in the auto sequence
	switch(autoPos) {
	default:
		autoSequence = "";
		printf("default\n");
		break;
	case kBlank:
		autoSequence = "";
		printf("blank\n");
		break;
	case kLeft:
		printf("Left position: ");
		if (switchSide == 'L') {
			printf("Left switch: ");
			autoSequence = "";
		} else if (switchSide == 'R') {
			printf("Right switch: ");
			autoSequence = "";
		}
		break;
	case kMiddle:
		printf("Middle Position: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = "";
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = "";
		}
		break;
	case kMiddleRight:
		printf("Middle Right: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = "";
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = "";
		}
		break;
	case kFarRight:
		printf("Far Right: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = "";
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = "";
		}
		break;
	case kIni:
		printf("Reading from Ini: ");
		if (switchSide == 'L') {
			printf("Left Switch: ");
			autoSequence = "";
		} else if (switchSide == 'R') {
			printf("Right Switch: ");
			autoSequence = "";
		}
		break;
	}

	const char* autoSequenceCharArray = autoSequence.c_str();
	printf("%s\n", autoSequenceCharArray);
//	QueueFromString(autoSequence);	// TODO uncomment this when ready
}

void CubeInSwitchMode::Init() {
	printf("Cube In Switch Mode Init\n");
//	currentCommand_->Init();
}

void CubeInSwitchMode::RefreshIni() {

}

CubeInSwitchMode::~CubeInSwitchMode() {
	// TODO Auto-generated destructor stub
}

