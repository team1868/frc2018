/*
 * BaselineMode.h
 *
 *  Created on: Feb 10, 2018
 *      Author: starr
 */

#ifndef SRC_AUTO_MODES_BASELINEMODE_H_
#define SRC_AUTO_MODES_BASELINEMODE_H_

#include "Auto/Modes/AutoMode.h"

class BaselineMode : public AutoMode{
public:
	BaselineMode(RobotModel *robot);
	void CreateQueue(string gameData, AutoMode::AutoPositions pos);
	void Init();
	virtual ~BaselineMode();
};

#endif /* SRC_AUTO_MODES_BASELINEMODE_H_ */
