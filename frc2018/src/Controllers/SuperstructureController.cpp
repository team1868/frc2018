#include <Controllers/SuperstructureController.h>

const double MAX_ELEVATOR_HEIGHT = 2; // TODO CHANGE THIS

SuperstructureController::SuperstructureController(RobotModel *myRobot, ControlBoard *myHumanControl) {
	robot_ = myRobot;
	humanControl_ = myHumanControl;

	currState_ = kInit;
	nextState_ = kIdle;

	intakeMotorOutput_ = 0.7; // TODO test this
	outtakeMotorOutput_ = -0.7; // TODO test this
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
		printf("kInit\n");
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
			 robot_->SetIntakeOutput(intakeMotorOutput_);
		} else if (humanControl_->GetOuttakeDesired()) {
			robot_->SetIntakeOutput(outtakeMotorOutput_);
		} else {
			robot_->SetIntakeOutput(0.0);
		}

		if (humanControl_->GetElevatorUpDesired()) {
			HoldCube();
			robot_->SetElevatorOutput(elevatorOutput_);
		} else if (humanControl_->GetElevatorDownDesired()) {
			HoldCube();
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
		robot_->SetIntakeOutput(intakeMotorOutput_);
	}
}

SuperstructureController::~SuperstructureController() {
	// TODO Auto-generated destructor stub
}

