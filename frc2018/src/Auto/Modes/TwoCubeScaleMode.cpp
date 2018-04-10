/*
 * TwoCubeScaleMode.cpp
 *
 *  Created on: Apr 9, 2018
 *      Author: Grace
 */

#include <Auto/Modes/TwoCubeScaleMode.h>

TwoCubeScaleMode::TwoCubeScaleMode(RobotModel *robot) : AutoMode(robot) {
	printf("In Two Cube In Scale Mode Constructor\n");
}

void TwoCubeScaleMode::CreateQueue(string gameData, AutoMode::AutoPositions pos) {

	AutoPositions autoPos = pos;
	printf("autopositions: %d\n", pos);
	char scaleSide = gameData[1];
	string autoSequence = "";

	switch(autoPos) {
		case kLeft:
			printf("Left position: ");
			if (scaleSide == 'L') {
				printf("Left scale: ");
				//autoSequence = "p d 21.25 e 5.2 t 70.0 o"; //last tested in competition
				autoSequence = "p d 21 e 5.4 t 80.0 o e -5.4 w 0 t 150 z 0.0"; //tested on KOP

			} else if (scaleSide == 'R') {
				printf("Right scale: ");
						//"d 17.5 t 90.0 t 90.0 p e 5.4 d 17.0 t -10.0 t -10.0 d 3.0 o"; // TODO test, angles should be in absolute
				if (gameData[0] == 'L') {
					autoSequence = "p d 12.4 e 1.8 t 90.0 d 2.3 w 0 o";
				} else {
					autoSequence = "d 17.5 t 90.0 t 90.0 d 17.0";
				}
			}
			break;
		case kFarRight:
			printf("Far Right: ");
			if (scaleSide == 'L') {
				printf("Left Scale: ");
	//			autoSequence = "d 17.5 t -90 t -90.0 p e 5.4 d 17.0 t 10.0 t 10.0 d 3.0 o"; // TODO test, angles should be in absolute
				if (gameData[0] == 'R') {
					autoSequence = "p d 12.4 e 1.8 t -90.0 d 2.3 w 0 o";
				} else {
					autoSequence = "d 17.5 t -90.0";
				}
			} else if (scaleSide == 'R') {
				printf("Right Scale: ");
				//autoSequence = "p d 21.25 e 5.2 t -70.0 o"; //tested in competition
				autoSequence = "p d 21 e 5.4 t -80.0 o e -5.4 w 0 t 150 z 0.0"; //tested on KOP

			}
			break;
		case kIni:
			printf("Reading from Ini: ");
			if (scaleSide == 'L') {
				printf("Left Switch: ");
				autoSequence = robot_->cubeInSwitchL_;
			} else if (scaleSide == 'R') {
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
		QueueFromString(autoSequence);
		printf("In Two Cube In Scale Mode Queue\n");
}

void TwoCubeScaleMode::Init() {
	printf("Two Cube In Scale Mode Init\n");
	currentCommand_->Init();
}

TwoCubeScaleMode::~TwoCubeScaleMode() {
	// TODO Auto-generated destructor stub
}

