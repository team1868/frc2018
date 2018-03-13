/*
 * TwoCubeInSwitchMode.h
 *
 *  Created on: Mar 12, 2018
 *      Author: Grace
 */

#ifndef SRC_AUTO_MODES_TWOCUBEINSWITCHMODE_H_
#define SRC_AUTO_MODES_TWOCUBEINSWITCHMODE_H_

#include "Auto/Modes/AutoMode.h"

class TwoCubeSwitchMode : public AutoMode {
public:
	TwoCubeSwitchMode(RobotModel *robot);
	void CreateQueue(string gameData, AutoMode::AutoPositions pos) override;
	void Init();
	virtual ~TwoCubeSwitchMode();
};

#endif /* SRC_AUTO_MODES_TWOCUBEINSWITCHMODE_H_ */
