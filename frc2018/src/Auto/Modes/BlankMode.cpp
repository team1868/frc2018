/*
 * BlankMode.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: Lynn D
 */

#include <Auto/Modes/BlankMode.h>

BlankMode::BlankMode() : AutoMode(NULL) {
	printf("In Blank Mode\n");
}

void BlankMode::CreateQueue(string gameData, AutoMode::AutoPositions pos) {
	printf("In Blank Mode Queue\n");
}

void BlankMode::Init() {
	printf("In Blank Mode Init\n");
}

BlankMode::~BlankMode() {
	// TODO Auto-generated destructor stub
}

