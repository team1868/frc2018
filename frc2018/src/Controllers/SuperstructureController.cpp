#include <Controllers/SuperstructureController.h>

//const double MAX_ELEVATOR_HEIGHT = 2; // TODO CHANGE THIS

SuperstructureController::SuperstructureController(RobotModel *myRobot, ControlBoard *myHumanControl) {
	robot_ = myRobot;
	humanControl_ = myHumanControl;

	currState_ = kInit;
	nextState_ = kIdle;

	elevatorOutput_ = 0.5; // TODO test this
	rampOutput_ = 0.75; // TODO test this
	rampReleaseTime_ = 0.0;
	rampReleaseDiffTime_ = 0.5;
	elevatorMovingCurr_ = false;
	elevatorMovingLast_ = false;
}

void SuperstructureController::Reset() {
	currState_ = kInit;
	nextState_ = kInit;

	robot_->SetIntakeOutput(0.0);
	robot_->SetElevatorOutput(0.0);
	rampReleaseTime_ = 0.0;
}

void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {
	switch(currState_) {
	case kInit:
		nextState_ = kIdle;
		robot_->SetIntakeOutput(0.0);
		robot_->SetElevatorOutput(0.0);
		robot_->EngageBrake();
		break;
	case kIdle:
		nextState_ = kIdle;
		if (humanControl_->GetWristUpDesired()) {
			robot_->SetWristUp();
		} else if (humanControl_->GetWristDownDesired()) {
			robot_->SetWristDown();
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
		} else if (humanControl_->GetIntakeHoldDesired()) {
			robot_->SetIntakeOutput(0.4);
		} else {
			robot_->SetIntakeOutput(0.0);
		}

		if (humanControl_->GetElevatorUpDesired()) { //elevator direction fixed
			printf("elevator up\n");
			if (!elevatorMovingLast_) {
				robot_->DisengageBrake();
				elevatorMovingCurr_ = true;
			}
			robot_->SetElevatorOutput(elevatorOutput_);
		} else if (humanControl_->GetElevatorDownDesired()) {
			printf("elevator down\n");
			if (!elevatorMovingLast_) {
				robot_->DisengageBrake();
				elevatorMovingCurr_ = true;
			}
			robot_->SetElevatorOutput(-elevatorOutput_);
		} else if (humanControl_->GetElevatorHoldDesired()) {
					printf("elevator down\n");
					robot_->SetElevatorOutput(-double(elevatorOutput_)/5.0);
		} else {
			robot_->SetElevatorOutput(0.0);
			if (elevatorMovingLast_) {
				robot_->EngageBrake();
			}
		}

		if (humanControl_->GetRampReleaseDesired()) {
			nextState_ = kRampRelease;
			rampReleaseTime_ = robot_->GetTime();
			robot_->ReleaseRampLegs();
		}
		break;
	case kRampRelease:
		printf("in kRampRelease\n");
		if (robot_->GetTime() - rampReleaseTime_ > 0.5) {
			robot_->ReleaseRamps();
			nextState_ = kRampRaise;
		} else {
			nextState_ = kRampRelease;
		}
		break;
	case kRampRaise:
		if (humanControl_->GetRampRaiseLDesired()) {
			robot_->SetRampMotorLOutput(rampOutput_);
		} else {
			robot_->SetRampMotorLOutput(0.0);
		}

		if (humanControl_->GetRampRaiseRDesired()) {
			robot_->SetRampMotorROutput(rampOutput_);
		} else {
			robot_->SetRampMotorROutput(0.0);
		}
		break;
	}
	elevatorMovingLast_ = elevatorMovingCurr_;
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

