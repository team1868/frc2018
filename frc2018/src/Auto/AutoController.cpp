#include <Auto/AutoController.h>

// Blank constructor
AutoController::AutoController() {
	autoMode = nullptr;
}

// Constructor that sets auto mode
AutoController::AutoController(AutoMode *myAutoMode){
	autoMode = myAutoMode;
}

// Setting auto mode
void AutoController::SetAutonomousMode(AutoMode *myAutoMode) {
	autoMode = myAutoMode;
}

void AutoController::Init(string gameData, AutoMode::AutoPositions pos) {
	printf("queuing now\n");
	if (autoMode == NULL) {
		printf("autoMode is null\n");
	} else {
		printf(gameData.c_str());
		autoMode->CreateQueue(gameData, pos);
		printf("Queue finished\n");
		autoMode->Init();
		printf("init finished\n");
	}
}

void AutoController::Update(double currTimeSec, double deltaTimeSec) {
	autoMode->Update(currTimeSec, deltaTimeSec);
}

bool AutoController::IsDone() {
	return autoMode->IsDone();
}
