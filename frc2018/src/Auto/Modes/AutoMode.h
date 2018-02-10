#ifndef SRC_AUTO_MODES_AUTOMODE_H_
#define SRC_AUTO_MODES_AUTOMODE_H_

#include "RobotModel.h"
#include "Auto/PIDSource/PIDInputSource.h"
#include "Auto/PIDSource/PIDOutputSource.h"
#include "Auto/Commands/AutoCommand.h"
#include "Auto/Commands/PivotCommand.h"
#include "Auto/Commands/DriveStraightCommand.h"

class AutoMode {
public:
	enum AutoPositions { kBlank, kLeft, kMiddle, kMiddleRight, kFarRight, kIni };

	AutoMode(RobotModel *robot, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoder) {
		printf("Constructing AutoMode\n");
		currentCommand_ = NULL;
		robot_ = robot;
		navX_ = navX;
		talonEncoder_ = talonEncoder;

		angleOutput_ = new AnglePIDOutput();
		distanceOutput_ = new DistancePIDOutput();
		printf("Done constructing AutoMode\n");
	};

	virtual ~AutoMode() {};

	virtual void CreateQueue(string gameData, AutoMode::AutoPositions pos) {
	}

	void QueueFromString(string autoSequence) {
		currentCommand_ = NULL;
		AutoCommand *lastCommand = NULL;
		RefreshIni();
		std::istringstream iss(autoSequence);
		bool breakDesired = false;
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
				printf("Angle: %f\n",angle);
				tempCommand = new PivotCommand(robot_, angle, true, navX_);
				break;
			case 'a':	// Drive straight at absolute position
				iss >> distance >> angle;
				printf("Distance: %f, Angle: %f\n", distance, angle);
				tempCommand = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, distance, angle);
				break;
			case 'd':	// Drive straight
				iss >> distance;
				printf("Distance: %f", distance);
				tempCommand = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, distance);
				break;
			default:	// When it's not listed, don't do anything :)
				currentCommand_ = NULL;
				tempCommand = NULL;
				breakDesired = true;
				break;
			}

			if (currentCommand_ == NULL) {
				currentCommand_ = tempCommand;
				lastCommand = currentCommand_;
			} else {
				lastCommand->SetNextCommand(tempCommand);
				lastCommand = lastCommand->GetNextCommand();			}
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

	virtual void RefreshIni() {

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
		printf("Successfully disabled\n");
	}

protected:
	AutoCommand *currentCommand_;
	RobotModel* robot_;

	NavXPIDSource* navX_;
	TalonEncoderPIDSource* talonEncoder_;

	AnglePIDOutput *angleOutput_;
	DistancePIDOutput *distanceOutput_;
};

#endif /* SRC_AUTO_MODES_AUTOMODE_H_ */
