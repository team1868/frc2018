/*
 * CubeInSwitchMode.h
 *
 *  Created on: Jan 12, 2018
 *      Author: Lynn D
 */

#ifndef SRC_AUTO_MODES_CUBEINSWITCHMODE_H_
#define SRC_AUTO_MODES_CUBEINSWITCHMODE_H_

#include "Auto/Modes/AutoMode.h"

class CubeInSwitchMode : public AutoMode{
public:
	CubeInSwitchMode(RobotModel *robot, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoder);
	void CreateQueue(string gameData, AutoPositions pos) override;
	void Init();
	void RefreshIni();

	virtual ~CubeInSwitchMode();
};

#endif /* SRC_AUTO_MODES_CUBEINSWITCHMODE_H_ */
