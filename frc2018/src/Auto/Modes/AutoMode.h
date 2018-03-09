#ifndef SRC_AUTO_MODES_AUTOMODE_H_
#define SRC_AUTO_MODES_AUTOMODE_H_

#include "Auto/Commands/AutoCommand.h"
#include "Auto/Commands/DriveStraightCommand.h"
#include "Auto/Commands/ElevatorHeightCommand.h"
#include "Auto/Commands/IntakeCommand.h"
#include "Auto/Commands/OuttakeCommand.h"
#include "Auto/Commands/ParallelCommand.h"
#include "Auto/Commands/PathCommand.h"
#include "Auto/Commands/PivotCommand.h"
#include "Auto/Commands/WristCommand.h"
#include "Auto/Commands/WaitingCommand.h"
#include "Auto/PIDSource/PIDInputSource.h"
#include "Auto/PIDSource/PIDOutputSource.h"
#include "RobotModel.h"
#include <iostream>

using namespace std;

class AutoMode {
public:
	enum AutoPositions { kBlank, kLeft, kMiddle, kMiddleRight, kFarRight, kIni };

	/**
	 * Constructs auto mode and initializes all variables.
	 */
	AutoMode(RobotModel *robot) {
		printf("Constructing AutoMode\n");

		firstCommand_ = NULL;
		currentCommand_ = NULL;
		robot_ = robot;
		navX_ = new NavXPIDSource(robot_);
		talonEncoder_ = new TalonEncoderPIDSource(robot_);
		angleOutput_ = new AnglePIDOutput();
		distanceOutput_ = new DistancePIDOutput();
		breakDesired_ = false;
		currAngle_ = robot_->GetNavXYaw();

		printf("Done constructing AutoMode\n");
	};

	virtual ~AutoMode() {};

	/**
	 * Creates the queue of commands. Currently empty until the auto mode inheriting overrides it.
	 */
	virtual void CreateQueue(string gameData, AutoMode::AutoPositions pos) {

	}

	/**
	 * Queues from a given string parameter. This accounts for parallel auto and calls GetCommand().
	 */
	void QueueFromString(string autoSequence) {
		firstCommand_ = NULL;
		currentCommand_ = NULL;
		AutoCommand *lastCommand = NULL;
		iss.str (autoSequence);
		cout << string ("autosequence" ) + autoSequence << endl;
		breakDesired_ = false;
		currAngle_ = robot_->GetNavXYaw();

		if (autoSequence == "") {
			printf("NO QUEUEUEUEUEUE!!!!");
		}

		//printf("AUto sequence: %s", autoSequence.c_str());

		while (!iss.eof() && !breakDesired_) {
			AutoCommand* tempCommand = NULL;
			char command;
			iss >> command;
			printf("Command: %c, ", command);

			if (command == 'p') {
				char charA, charB;
				iss >> charA;
				printf("CommandA %c ", charA);
				AutoCommand* commandA = GetStringCommand(charA);
				iss >> charB;
				printf("CommandB %c ", charB);
				AutoCommand* commandB = GetStringCommand(charB);
				tempCommand = new ParallelCommand(commandA, commandB);
			} else {
				tempCommand = GetStringCommand(command);
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
		iss.clear();

	}

	/**
	 * Creates a new command according to the command character input and inputStringStream.
	 */
	AutoCommand* GetStringCommand(char command) {
		AutoCommand* tempCommand = NULL;

		switch(command) {
		case 't':	// Pivots with absolute position
			double angle;
			iss >> angle;
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				currAngle_ = angle;
				printf("Angle: %f\n", angle);
				tempCommand = new PivotCommand(robot_, angle, true, navX_);
			}
			break;
		case 'd':	// Drive straight
			double distance;
			iss >> distance;
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				printf("Distance: %f\n", distance);
				tempCommand = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, distance, currAngle_);
			}
			break;
		case 'i':
			double intakeOutput;
			iss >> intakeOutput;
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new IntakeCommand(robot_, intakeOutput);
			}
			break;
		case 'o':   // Outtake
			printf("Outtake Command\n");
			tempCommand = new OuttakeCommand(robot_);
			break;
		case 'e':
			printf("Elevator Command\n");
			double height;
			iss >> height;
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new ElevatorHeightCommand(robot_, height);
			}
			break;
		case 'w':
			printf("Wrist Command\n");
			int wUp; // wrist up
			iss >> wUp;
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				if (wUp == 1) {
					tempCommand = new WristCommand(robot_, true);
				} else {
					tempCommand = new WristCommand(robot_, false);
				}
			}
			break;
		case 's':
			printf("Wait Command\n");
			double waitTime;
			iss >> waitTime;
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new WaitingCommand(waitTime);
			}
			break;
		default:	// When it's not listed, don't do anything :)
			printf("Unexpected character %c detected. Terminating queue", command);
			firstCommand_ = NULL;
			currentCommand_ = NULL;
			tempCommand = NULL;
			breakDesired_ = true;
			break;
		}

		return tempCommand;
	}

	bool IsFailed(char command) {
		if (iss.fail()) {
			iss.clear();
			printf("Unexpected character detected after %c. Terminating queue", command);
			firstCommand_ = NULL;
			currentCommand_ = NULL;
			breakDesired_ = true;
			return true;
		}
		return false;
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
				currentCommand_->Reset();
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
			printf("Resetting current command\n");
			currentCommand_->Reset();
		}
		if (firstCommand_ != NULL) {
			currentCommand_ = firstCommand_;
			AutoCommand* nextCommand;
			while (currentCommand_ != NULL) {
				nextCommand = currentCommand_->GetNextCommand();
				delete(currentCommand_);
				currentCommand_ = nextCommand;
			}
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

	istringstream iss;
	bool breakDesired_;
	double currAngle_;
};

#endif /* SRC_AUTO_MODES_AUTOMODE_H_ */
