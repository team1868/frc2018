/*
 * TestMode.h
 *
 *  Created on: Jan 14, 2018
 *      Author: Lynn D
 */

#ifndef SRC_AUTO_MODES_TESTMODE_H_
#define SRC_AUTO_MODES_TESTMODE_H_

#include "RobotModel.h"
#include "Auto/Modes/AutoMode.h"
#include "Auto/PIDSource/PIDInputSource.h"
#include "Auto/PIDSource/PIDOutputSource.h"
#include "Auto/Commands/PivotCommand.h"
#include "Auto/Commands/DriveStraightCommand.h"

class TestMode : public AutoMode {
public:
	TestMode(RobotModel *robot, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoder);
	void CreateQueue();
	void Init();
	void RefreshIni();

	virtual ~TestMode();

private:
	RobotModel *robot_;
	NavXPIDSource *navX_;
	TalonEncoderPIDSource *talonEncoder_;

	AnglePIDOutput *angleOutput_;
	DistancePIDOutput *distanceOutput_;

	PivotCommand *pivot_;
	PivotCommand *pivotSecond_;
	PivotCommand *pivotThird_;
	PivotCommand *pivotFourth_;
	DriveStraightCommand *driveStraightFirst_;
	DriveStraightCommand *driveStraightSecond_;
	DriveStraightCommand *driveStraightThird_;
	DriveStraightCommand *driveStraightFourth_;

	AutoCommand *firstCommand_;
};

#endif /* SRC_AUTO_MODES_TESTMODE_H_ */
