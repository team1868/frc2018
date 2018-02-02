/*
 * SequenceMode.h
 *
 *  Created on: Jan 26, 2018
 *      Author: alisha
 */

#ifndef SRC_AUTO_MODES_SEQUENCEMODE_H_
#define SRC_AUTO_MODES_SEQUENCEMODE_H_

#include "RobotModel.h"
#include "Auto/Modes/AutoMode.h"
#include "Auto/PIDSource/PIDInputSource.h"
#include "Auto/PIDSource/PIDOutputSource.h"
#include "Auto/Commands/PivotCommand.h"
#include "Auto/Commands/DriveStraightCommand.h"

class SequenceMode : public AutoMode{
public:
	SequenceMode(RobotModel *robot, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoder);
	void CreateQueue();
	void Init();
	void RefreshIni();
	virtual ~SequenceMode();

private:
	RobotModel *robot_;
	NavXPIDSource *navX_;
	TalonEncoderPIDSource *talonEncoder_;

	AnglePIDOutput *angleOutput_;
	DistancePIDOutput *distanceOutput_;

	AutoCommand *firstCommand_;
	AutoCommand *lastCommand_;

	std::string autoSequenceStr_;
};

#endif /* SRC_AUTO_MODES_SEQUENCEMODE_H_ */
