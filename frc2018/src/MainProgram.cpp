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

		// Setup to chooser auto mode from SmartDashboard

//		autoChooser_.AddDefault("Blank Auto", new BlankMode());

		autoChooser_.AddDefault("Baseline Mode", new BaselineMode(robot_));

		autoChooser_.AddObject("Switch Mode", new CubeInSwitchMode(robot_));
		autoChooser_.AddObject("Test Mode", new TestMode(robot_));

		autoChooser_.AddObject("KOP Test", new KOPTestMPMode(robot_));

		SmartDashboard::PutData("Auto Modes", &autoChooser_);
		autoMode_ = autoChooser_.GetSelected();
		if (autoMode_ == NULL) {
			printf("autoMode_ is null in RobotInit\n");
		}
		autoPosition_ = humanControl_->GetDesiredAutoPosition();
		ResetTimerVariables();
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
		robot_->ZeroNavXYaw();
		ResetTimerVariables();
		string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if(gameData == "") {
			gameData = "LRL";
		}
		robot_->RefreshIni();

		autoMode_ = autoChooser_.GetSelected();
		printf("hi\n");
//		autoMode_ = new CubeInSwitchMode(robot_);
		if (autoMode_ == NULL) {
			printf("auto mode is null from autoinit\n");
		} else {
			printf("Get selected\n");
			autoController_->SetAutonomousMode(autoMode_);
			printf("Auto mode set\n");
			autoController_->Init(gameData, autoPosition_);
			printf("Auto mode init\n");
		}
		robot_->ResetTimer();
	}

	void AutonomousPeriodic() {
		UpdateTimerVariables();
		autoController_->Update(currTimeSec_, deltaTimeSec_);
	}

	void TeleopInit() {
		robot_->ResetTimer();
		ResetTimerVariables();
		ResetControllers();
	}

	void TeleopPeriodic() {
		UpdateTimerVariables();
		humanControl_->ReadControls();
		driveController_->Update(currTimeSec_, deltaTimeSec_);
		superstructureController_->Update(currTimeSec_, deltaTimeSec_);
		Logger::LogState(robot_, humanControl_);
	}

	void TestPeriodic() {}

	void DisabledInit() {
		if (autoMode_ != NULL) {
			autoMode_->Disable();
		}

		robot_->RefreshIni();
		robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
	}
	void DisabledPeriodic() {
		humanControl_->ReadControls();
		autoPosition_ = humanControl_->GetDesiredAutoPosition();
		SmartDashboard::PutNumber("NavX Yaw: ", robot_->GetNavXYaw());
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
};

START_ROBOT_CLASS(MainProgram)
