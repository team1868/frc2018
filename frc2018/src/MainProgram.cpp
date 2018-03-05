/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <WPILib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "RobotModel.h"
#include "DriverStation/ControlBoard.h"
#include "Controllers/DriveController.h"
#include "Controllers/SuperstructureController.h"
#include "Auto/AutoController.h"
#include "Auto/PIDSource/PIDInputSource.h"
#include "Auto/PIDSource/PIDOutputSource.h"

#include "Logger.h"

using namespace std;

class MainProgram : public frc::IterativeRobot {

public:
	void RobotInit() {
		printf("In robot init\n");
		// Initializing robot
		robot_ = new RobotModel();
		robot_->ZeroNavXYaw();
		robot_->RefreshIni();

		humanControl_ = new ControlBoard();
		driveController_ = new DriveController(robot_, humanControl_);
		superstructureController_ = new SuperstructureController(robot_, humanControl_);

		// Initializing auto controller
		autoController_ = new AutoController();

		autoMode_ = NULL;

		// Setup to set automode from ini file
		switch (robot_->autoMode_) {
		case 0:
			autoMode_ = new BaselineMode(robot_);
			printf("BASELINE AUTO");
			break;
		case 1:
			autoMode_  = new CubeInSwitchMode(robot_);
			printf("CUBE IN SWITCH AUTO");
			break;
		case 2:
			autoMode_ =  new TestMode(robot_);
			printf("TEST AUTO");
			break;
		case 3:
			autoMode_ = new KOPTestMPMode(robot_);
			printf("KOP TEST MP AUTO");
			break;
		default:
			autoMode_ = new BlankMode(robot_);
			printf("BLANK AUTO");
			break;
		}

		// Setup to chooser auto mode from SmartDashboard
		/*
		AutoMode *blank = new BlankMode(robot_);
		autoChooser_.AddDefault("Blank Auto", blank);
		printf("blank %x \n", blank);

		AutoMode *baseline = new BaselineMode(robot_);
		autoChooser_.AddObject("Baseline Mode", baseline);
		printf("baseline %x \n", baseline);

		autoChooser_.AddObject("Switch Mode", new CubeInSwitchMode(robot_));
		autoChooser_.AddObject("Test Mode", new TestMode(robot_));

		autoChooser_.AddObject("KOP Test", new KOPTestMPMode(robot_));
		autoChooser_.AddObject("RSideRSwitchMP", new RightSideToRightSwitchMode(robot_));
		autoChooser_.AddObject("LSideLSwitchMP", new RightSideToRightSwitchMode(robot_));


		SmartDashboard::PutData("Auto Modes", &autoChooser_);
		*/

//		if (autoMode_ == NULL) {
//			printf("autoMode_ is null in RobotInit\n");
//		}

		//setting auto position from ini file

		switch (robot_->autoPos_) {
		case 0:
			autoPosition_ = AutoMode::kLeft;
			printf("auto position LEFT");
			break;
		case 1:
			autoPosition_ = AutoMode::kMiddle;
			printf("auto position MIDDLE");
			break;
		case 2:
			autoPosition_ = AutoMode::kMiddleRight;
			printf("auto position RIGHT");
			break;
		case 3:
			autoPosition_ = AutoMode::kFarRight;
			printf("auto position FAR RIGHT");
			break;
		case 4:
			autoPosition_ = AutoMode::kBlank;
			printf("auto position BLANK");
			break;
		default:
			autoPosition_ = AutoMode::kLeft;
			printf("auto position LEFT FROM DEFAULT");
			break;
		}


		//UNCOMMENT autoPosition if driverstation is set up for testing!!!
		//autoPosition_ = humanControl_->GetDesiredAutoPosition();
		ResetTimerVariables();

		robot_->SetWristUp();
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard.
	 *
	 * Auto Position List (Determined by the auto switches):
	 * 0: Far Left
	 * 1: Middle
	 * 2: Middle Right
	 * 3: Far Right
	 */
	void AutonomousInit() override {
		robot_->ResetTimer();
		robot_->SetTalonBrakeMode();
		robot_->ZeroNavXYaw();
		robot_->SetHighGear();
		robot_->StopCompressor();
		robot_->GetElevatorEncoder()->Reset(); // START ELEVATOR AT ZERO
		ResetTimerVariables();
		autoModeSet_ = false;
		gameData_ = "";

		autoMode_ = new TestMode(robot_);
		printf("hi\n");

		printf("Setting autonomous mode %x \n", autoMode_);
		autoController_->SetAutonomousMode(autoMode_);

		if (!GameDataSet()) {
			GetGameMessage();
		}

		if (GameDataSet()) {
			SetAutoMode();
		}

		robot_->ResetTimer();
	}

	void AutonomousPeriodic() {
		robot_->PrintState();
		if (!GameDataSet()) {
			GetGameMessage();
		}

		if (GameDataSet() && !autoModeSet_) {
			printf("Setting autonomous mode\n");
			autoController_->SetAutonomousMode(autoMode_);
			SetAutoMode();
		}

		UpdateTimerVariables();
		autoController_->Update(currTimeSec_, deltaTimeSec_);
		if (autoController_->IsDone()) {
			robot_->StartCompressor();
		}
	}

	void TeleopInit() {
		robot_->ResetTimer();
		robot_->SetTalonCoastMode();
		ResetTimerVariables();
		ResetControllers();
		robot_->StartCompressor();
	}

	void TeleopPeriodic() {
		UpdateTimerVariables();
		robot_->PrintState();
		humanControl_->ReadControls();
		driveController_->Update(currTimeSec_, deltaTimeSec_);
		superstructureController_->Update(currTimeSec_, deltaTimeSec_);
		Logger::LogState(robot_, humanControl_);
		SmartDashboard::PutNumber("Intake", robot_->intakeMotorOutput_);
		SmartDashboard::PutNumber("Outtake", robot_->outtakeMotorOutput_);
	}

	void TestPeriodic() {}

	void DisabledInit() {
		robot_->SetTalonCoastMode();
		if (autoMode_ != NULL) {
			autoMode_->Disable();
		}

		robot_->RefreshIni();
		robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
		robot_->ZeroNavXYaw();
		robot_->ResetDriveEncoders();
	}
	void DisabledPeriodic() {
		humanControl_->ReadControls();
		//uncomment if connected to driverstation and auto switches finished!
		//autoPosition_ = humanControl_->GetDesiredAutoPosition();
		robot_->PrintState();
		SmartDashboard::PutNumber("Intake", robot_->intakeMotorOutput_);
		SmartDashboard::PutNumber("Outtake", robot_->outtakeMotorOutput_);
	};
private:
	// Robot setup
	RobotModel *robot_;
	ControlBoard *humanControl_;
	DriveController *driveController_;
	SuperstructureController *superstructureController_;

	// Auto setup
	AutoController *autoController_;
	AutoMode *autoMode_;
	frc::SendableChooser<AutoMode*> autoChooser_;
	AutoMode::AutoPositions autoPosition_;
	string gameData_;
	bool autoModeSet_;

	// Time setup
	double currTimeSec_;
	double lastTimeSec_;
	double deltaTimeSec_;

	void ResetTimerVariables() {
		currTimeSec_ = robot_->GetTime();
		lastTimeSec_ = currTimeSec_;
		deltaTimeSec_ = 0.0;
	}

	void UpdateTimerVariables() {
		lastTimeSec_ = currTimeSec_;
		currTimeSec_ = robot_->GetTime();
		deltaTimeSec_ = currTimeSec_ - lastTimeSec_;
	}

	void ResetControllers() {
		driveController_->Reset();
		superstructureController_->Reset();
	}

	void GetGameMessage() {
		gameData_ = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	}
	void SetAutoMode() {
		robot_->RefreshIni();

		//autoMode_ = autoChooser_.GetSelected();
		printf("in top of Set Auto Mode\n");
		//		autoMode_ = new CubeInSwitchMode(robot_);
		if (autoMode_ == NULL) {
			printf("auto mode is null from autoinit\n");
		} else {
			printf("Auto mode set\n");
			autoController_->Init(gameData_, autoPosition_);
			printf("Auto mode init\n");
			autoModeSet_ = true;
		}

	}

	bool GameDataSet() {
		return (gameData_.length() > 0);
	}
};

START_ROBOT_CLASS(MainProgram)
