#include <Controllers/SuperstructureController.h>

//const double MAX_ELEVATOR_HEIGHT = 2; // TODO CHANGE THIS

SuperstructureController::SuperstructureController(RobotModel *myRobot, ControlBoard *myHumanControl) {
	robot_ = myRobot;
	humanControl_ = myHumanControl;

	currState_ = kInit;
	nextState_ = kIdle;

	//elevatorOutput_ = 0.5; // TODO test this	// 0.7 is sad with cube :( no chicken tenders sry
	elevatorUpOutput_ = robot_->elevatorOutput_;
	elevatorDownOutput_ = robot_->elevatorOutput_;
	elevatorMaxOutput_ = robot_->elevatorMaxOutput_;
	elevatorRamp_ = robot_->elevatorRampRate_;

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

	elevatorUpOutput_ = robot_->elevatorOutput_;
	elevatorDownOutput_ = robot_->elevatorOutput_;
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
			if ((robot_->GetElevatorCurrent() > elevatorCurrentLimit_) || elevatorCurrLimitReached_) {
				elevatorCurrLimitReached_ = true;
				robot_->SetElevatorOutput(0.0);
			} else {
				if (elevatorUpOutput_ < elevatorMaxOutput_) {
					elevatorUpOutput_ *= elevatorRamp_;
				}
				robot_->SetElevatorOutput(elevatorUpOutput_);
				elevatorCurrLimitReached_ = false;
			}
		} else if (humanControl_->GetElevatorDownDesired()) {
			if ((robot_->GetElevatorCurrent() > elevatorCurrentLimit_) || elevatorCurrLimitReached_) {
				elevatorCurrLimitReached_ = true;
				robot_->SetElevatorOutput(0.0);
			} else {
				robot_->SetElevatorOutput(-elevatorDownOutput_);
				elevatorCurrLimitReached_ = false;
			}
		} else {
			robot_->SetElevatorOutput(0.0);
			elevatorUpOutput_ = robot_->elevatorOutput_;
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

SuperstructureController::~SuperstructureController() {
	// TODO Auto-generated destructor stub
}

