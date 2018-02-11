/*
 * TestMode.h
 *
 *  Created on: Jan 14, 2018
 *      Author: Lynn D
 */

#ifndef SRC_AUTO_MODES_TESTMODE_H_
#define SRC_AUTO_MODES_TESTMODE_H_

#include "Auto/Modes/AutoMode.h"

class TestMode : public AutoMode {
public:
	TestMode(RobotModel *robot);
	void CreateQueue(string gameData, AutoMode::AutoPositions pos) override;
	void Init();

	virtual ~TestMode();

private:
	DriveStraightCommand* driveStraightFirst_;
	PivotCommand* pivot_;
	DriveStraightCommand* driveStraightSecond_;
	PivotCommand* pivotSecond_;
	DriveStraightCommand* driveStraightThird_;
	PivotCommand* pivotThird_;
	DriveStraightCommand* driveStraightFourth_;
	PivotCommand* pivotFourth_;
};

#endif /* SRC_AUTO_MODES_TESTMODE_H_ */
