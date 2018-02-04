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
			printf("Left switch\n");
			autoSequence = "";
		} else if (switchSide == 'R') {

		}
		break;
	case kMiddle:
		printf("middle\n");
		if (switchSide == 'L') {

		} else if (switchSide == 'R') {

		}
		break;
	case kMiddleRight:
		printf("middle right\n");
		if (switchSide == 'L') {

		} else if (switchSide == 'R') {

		}
		break;
	case kFarRight:
		printf("far right\n");
		if (switchSide == 'L') {

		} else if (switchSide == 'R') {

		}
		break;
	}

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

