/*
 * CubeInSwitchMode.h
 *
 *  Created on: Feb 12, 2018
 *      Author: starr
 */

#ifndef SRC_AUTO_MODES_CUBEINSWITCHMODE_H_
#define SRC_AUTO_MODES_CUBEINSWITCHMODE_H_

#include "Auto/Modes/AutoMode.h"

class CubeInSwitchMode : public AutoMode {
public:
	CubeInSwitchMode(RobotModel *robot);
	void CreateQueue(string gameData, AutoMode::AutoPositions pos) override;
	void Init();
	virtual ~CubeInSwitchMode();
};

#endif /* SRC_AUTO_MODES_CUBEINSWITCHMODE_H_ */
