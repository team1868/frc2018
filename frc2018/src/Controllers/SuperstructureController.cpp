#include <Controllers/SuperstructureController.h>

const double MAX_ELEVATOR_HEIGHT = 2; // TODO CHANGE THIS

SuperstructureController::SuperstructureController(RobotModel *myRobot, ControlBoard *myHumanControl) {
	robot_ = myRobot;
	humanControl_ = myHumanControl;

	currState_ = kInit;
	nextState_ = kIdle;

	elevatorOutput_ = 0.5; // TODO test this
}

void SuperstructureController::Reset() {
	currState_ = kInit;
	nextState_ = kInit;

	robot_->SetIntakeOutput(0.0);
	robot_->SetElevatorOutput(0.0);
}

void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {
	switch(currState_) {
	case kInit:
		nextState_ = kIdle;
		robot_->SetIntakeOutput(0.0);
		robot_->SetElevatorOutput(0.0);
		break;
	case kIdle:
		nextState_ = kIdle;
		if (humanControl_->GetWristDesired()) {
			if (robot_->GetWristUp()) {
				robot_->SetWristDown();
			} else {
				robot_->SetWristUp();
			}
		}
		if (humanControl_->GetHoldCubeDesired()) {
			HoldCube();
		}

		if (humanControl_->GetIntakeDesired()) {
			printf("intaking\n");
			robot_->SetIntakeOutput(robot_->intakeMotorOutput_);
		} else if (humanControl_->GetOuttakeDesired()) {
			printf("outtaking\n");
			robot_->SetIntakeOutput(robot_->outtakeMotorOutput_);
		} else {
			robot_->SetIntakeOutput(0.0);
		}

		if (humanControl_->GetElevatorUpDesired()) {
			printf("elevator up\n");
			robot_->SetElevatorOutput(elevatorOutput_);
		} else if (humanControl_->GetElevatorDownDesired()) {
			printf("elevator down\n");
			robot_->SetElevatorOutput(-elevatorOutput_);
		} else {
			robot_->SetElevatorOutput(0.0);
		}

		if (humanControl_->GetRampDesired()) {
			nextState_ = kRamp;
		}
		printf("in kIdle\n");
		break;
	case kRamp:
		printf("in kRamp\n");
		nextState_ = kRamp;
		break;
	}
	currState_ = nextState_;
}

void SuperstructureController::RefreshIni() {

}

void SuperstructureController::HoldCube() {
	if (robot_->GetCubeInIntake()) {
		robot_->SetIntakeOutput(0.0);
	} else {
		robot_->SetIntakeOutput(robot_->intakeMotorOutput_);
	}
}

SuperstructureController::~SuperstructureController() {
	// TODO Auto-generated destructor stub
}

