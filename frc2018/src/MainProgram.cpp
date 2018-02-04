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

	// Robot setup
	RobotModel *robot_;
	ControlBoard *humanControl_;
	DriveController *driveController_;
	SuperstructureController *superstructureController_;

	// Auto setup
	AutoController *autoController_;
	AutoMode *autoMode_;
	frc::SendableChooser<AutoMode*> autoChooser_;

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonEncoderSource_;

	// Time setup
	Timer *timer_;
	double currTimeSec_;
	double lastTimeSec_;
	double deltaTimeSec_;

public:
	void RobotInit() {
		// Initializing robot
		robot_ = new RobotModel();
		robot_->ZeroNavXYaw();
		robot_->RefreshIni();

		humanControl_ = new ControlBoard();
		driveController_ = new DriveController(robot_, humanControl_);
		superstructureController_ = new SuperstructureController(robot_, humanControl_);

		// PID sources
		navXSource_ = new NavXPIDSource(robot_);
		talonEncoderSource_ = new TalonEncoderPIDSource(robot_);
		timer_ = new Timer();

		// Initializing auto controller
		autoController_ = new AutoController();

		// Setup to chooser auto mode from SmartDashboard
		autoChooser_.AddDefault("Blank Auto", new BlankMode());
		autoChooser_.AddObject("One Cube in Switch Mode", new CubeInSwitchMode(robot_, navXSource_, talonEncoderSource_));
		autoChooser_.AddObject("Test Mode", new TestMode(robot_, navXSource_, talonEncoderSource_));
//		autoChooser_.AddObject("Sequence Mode", new SequenceMode(robot_, navXSource_, talonEncoderSource_));
		SmartDashboard::PutData("Auto Modes", &autoChooser_);

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
		string gameData = "LRL"; // TODO Change this :)
//		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		robot_->RefreshIniVals();

		autoMode_ = autoChooser_.GetSelected();
		autoMode_ = new CubeInSwitchMode(robot_, navXSource_, talonEncoderSource_);
		if (autoMode_ == NULL) {
			printf("auto mode is null from autoinit\n");
		}
		printf("Get selected\n");
		autoController_->SetAutonomousMode(autoMode_);
		printf("Auto mode set\n");
		autoController_->Init(gameData, autoPosition_);
		printf("Auto mode init\n");

		// TAKE OUT AFTER DONE
		robot_->ZeroNavXYaw();
	}

	void AutonomousPeriodic() {
		autoController_->Update(currTimeSec_, deltaTimeSec_);
	}

	void TeleopInit() {
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

		robot_->RefreshIniVals();
	}
	void DisabledPeriodic() {
		SmartDashboard::PutNumber("NavX Yaw: ", robot_->GetNavXYaw());
	};
private:
	AutoMode::AutoPositions autoPosition_;

	void ResetTimerVariables() {
		currTimeSec_ = 0.0;
		lastTimeSec_ = 0.0;
		deltaTimeSec_ = 0.0;
	}

	void UpdateTimerVariables() {
		lastTimeSec_ = currTimeSec_;
		currTimeSec_ = robot_->GetTime();
		deltaTimeSec_ = currTimeSec_ - lastTimeSec_;
	}

	void ResetControllers() {
		driveController_->Reset();
	}
};

START_ROBOT_CLASS(MainProgram)
