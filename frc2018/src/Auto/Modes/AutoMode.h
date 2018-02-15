#ifndef SRC_AUTO_MODES_AUTOMODE_H_
#define SRC_AUTO_MODES_AUTOMODE_H_

#include "RobotModel.h"
#include "Auto/PIDSource/PIDInputSource.h"
#include "Auto/PIDSource/PIDOutputSource.h"
#include "Auto/Commands/AutoCommand.h"
#include "Auto/Commands/PivotCommand.h"
#include "Auto/Commands/DriveStraightCommand.h"
#include "Auto/Commands/PathCommand.h"
#include "Auto/Commands/OuttakeCommand.h"


class AutoMode {
public:
	enum AutoPositions { kBlank, kLeft, kMiddle, kMiddleRight, kFarRight, kIni };

	AutoMode(RobotModel *robot) {
		printf("Constructing AutoMode\n");
		firstCommand_ = NULL;
		currentCommand_ = NULL;
		robot_ = robot;
		navX_ = new NavXPIDSource(robot_);
		talonEncoder_ = new TalonEncoderPIDSource(robot_);
		angleOutput_ = new AnglePIDOutput();
		distanceOutput_ = new DistancePIDOutput();
		printf("Done constructing AutoMode\n");
	};

	virtual ~AutoMode() {};

	virtual void CreateQueue(string gameData, AutoMode::AutoPositions pos) {

	}

	void QueueFromString(string autoSequence) {
		firstCommand_ = NULL;
		currentCommand_ = NULL;
		AutoCommand *lastCommand = NULL;
		std::istringstream iss(autoSequence);
		bool breakDesired = false;
		double currentAngle = robot_->GetNavXYaw();

		while (!iss.eof() && !breakDesired) {
			AutoCommand* tempCommand = NULL;
			char command;
			iss >> command;
			printf("Command: %c, ", command);

			double angle;
			double distance;

			switch(command) {
			case 'p':	// Pivots with absolute position
				iss >> angle;
				currentAngle = angle;
				printf("Angle: %f\n",angle);
				tempCommand = new PivotCommand(robot_, angle, true, navX_);
				break;
			case 'd':	// Drive straight
				iss >> distance;
				printf("Distance: %f\n", distance);
				tempCommand = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, distance, currentAngle);
				break;
			case 'o':   // Outtake
				printf("Outtake Command\n");
				tempCommand = new OuttakeCommand(robot_);
				break;
			default:	// When it's not listed, don't do anything :)
				printf("Unexpected character %c detected. Terminating queue", command);
				firstCommand_ = NULL;
				currentCommand_ = NULL;
				tempCommand = NULL;
				breakDesired = true;
				break;
			}

			if (firstCommand_ == NULL) {
				firstCommand_ = tempCommand;
				currentCommand_ = firstCommand_;
				lastCommand = currentCommand_;
			} else {
				lastCommand->SetNextCommand(tempCommand);
				lastCommand = lastCommand->GetNextCommand();
			}
		}
	}

	virtual void Init() = 0;

	/**
	 * Updates the current command and prints out if each command is complete and also
	 * if the automode update is finished
	 * @param currTimeSec a double current time
	 * @param deltaTimeSec a double how often to update
	 */
	void Update(double currTimeSec, double deltaTimeSec) {
		if (currentCommand_ != NULL) {
			if (currentCommand_->IsDone()) {
//				DO_PERIODIC(1, printf("Command complete at: %f \n", currTimeSec));
				currentCommand_ = currentCommand_->GetNextCommand();
				if (currentCommand_ != NULL) {
//					DO_PERIODIC(1, printf("Command start at: %f \n", currTimeSec));
					currentCommand_->Init();
					printf("Initializing current commmand\n");
				}
			} else {
				currentCommand_->Update(currTimeSec, deltaTimeSec);
			}
		} else {
//			printf("Done with auto mode update\n");
		}
	}

	/**
	 * Returns when AutoMode is done
	 * @return true if there is no current command or current command is NULL
	 */
	bool IsDone() {
		return (currentCommand_ == NULL);
	}

	void Disable(){
		printf("Disabling\n");
		if (!IsDone()) {
			currentCommand_->Reset();
		}
		currentCommand_ = firstCommand_;
		AutoCommand* nextCommand;
		while (currentCommand_ != NULL) {
			nextCommand = currentCommand_->GetNextCommand();
			delete(currentCommand_);
			currentCommand_ = nextCommand;
		}

		printf("Successfully disabled\n");
	}

protected:
	AutoCommand *firstCommand_;
	AutoCommand *currentCommand_;
	RobotModel* robot_;

	NavXPIDSource* navX_;
	TalonEncoderPIDSource* talonEncoder_;

	AnglePIDOutput *angleOutput_;
	DistancePIDOutput *distanceOutput_;
};

#endif /* SRC_AUTO_MODES_AUTOMODE_H_ */
