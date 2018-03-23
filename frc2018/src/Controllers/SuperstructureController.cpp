#include <Controllers/SuperstructureController.h>

//const double MAX_ELEVATOR_HEIGHT = 2; // TODO CHANGE THIS

SuperstructureController::SuperstructureController(RobotModel *myRobot, ControlBoard *myHumanControl) {
	robot_ = myRobot;
	humanControl_ = myHumanControl;

	currState_ = kInit;
	nextState_ = kIdle;

	//elevatorOutput_ = 0.5; // TODO test this	// 0.7 is sad with cube :( no chicken tenders sry
	elevatorOutput_ = robot_->elevatorOutput_;
	rampOutput_ = 0.75; // TODO test this
	rampReleaseTime_ = 0.0;
	rampReleaseDiffTime_ = 0.5;
	elevatorCurrentLimit_ = 50;
	elevatorMovingCurr_ = false;
	elevatorMovingLast_ = false;
	elevatorCurrLimitReached_ = false;
}

void SuperstructureController::Reset() {
	currState_ = kInit;
	nextState_ = kInit;

	robot_->SetIntakeOutput(0.0);
	robot_->SetElevatorOutput(0.0);
	rampReleaseTime_ = 0.0;

	elevatorOutput_ = robot_->elevatorOutput_;
	elevatorMovingCurr_ = false;
	elevatorMovingLast_ = false;
	elevatorCurrLimitReached_ = false;
}

void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {
	switch(currState_) {
	case kInit:
		nextState_ = kIdle;
		robot_->SetIntakeOutput(0.0);
		robot_->SetElevatorOutput(0.0);
		robot_->EngageBrake();
		elevatorMovingCurr_ = false;
		elevatorMovingLast_ = false;
		elevatorCurrLimitReached_ = false;
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
//			robot_->SetIntakeOutput(robot_->intakeMotorOutput_);
			robot_->SetIntakeOutput((robot_->intakeMotorOutput_), (robot_->intakeMotorOutput_ - robot_->intakeMotorOutputSubtract_));
		} else if (humanControl_->GetOuttakeDesired()) {
			printf("outtaking\n");
			robot_->SetIntakeOutput(robot_->outtakeMotorOutput_);
		} else if (humanControl_->GetOuttakeFastDesired()) {
			printf("outtaking fast\n");
			robot_->SetIntakeOutput(robot_->outtakeFastMotorOutput_);
		} else if (humanControl_->GetIntakeHoldDesired()) {
			robot_->SetIntakeOutput(0.3);
		} else {
			robot_->SetIntakeOutput(0.0);
		}

		if (humanControl_->GetElevatorUpDesired()) { //elevator direction fixed
//			if (!elevatorMovingLast_) {
//				printf("Disengage Brake\n");
//				robot_->DisengageBrake();
//				elevatorMovingCurr_ = true;
//			}
			if ((robot_->GetElevatorCurrent() > elevatorCurrentLimit_) || elevatorCurrLimitReached_) {
				elevatorCurrLimitReached_ = true;
				robot_->SetElevatorOutput(0.0);
			} else {
				robot_->SetElevatorOutput(elevatorOutput_);
				elevatorCurrLimitReached_ = false;
			}
		} else if (humanControl_->GetElevatorDownDesired()) {
//			if (!elevatorMovingLast_) {
//				printf("Disengage Brake\n");
//				robot_->DisengageBrake();
//				elevatorMovingCurr_ = true;
//			}
			if ((robot_->GetElevatorCurrent() > elevatorCurrentLimit_) || elevatorCurrLimitReached_) {
				elevatorCurrLimitReached_ = true;
				robot_->SetElevatorOutput(0.0);
			} else {
				robot_->SetElevatorOutput(-elevatorOutput_);
				elevatorCurrLimitReached_ = false;
			}
		} else if (humanControl_->GetElevatorHoldDesired()) {
			robot_->SetElevatorOutput(-double(elevatorOutput_)/5.0);
		} else {
			robot_->SetElevatorOutput(0.0);
//			if (elevatorMovingLast_) {
//				printf("Engage Brake\n");
//				robot_->EngageBrake();
//			}
			elevatorMovingCurr_ = false;
			elevatorCurrLimitReached_ = false;
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

