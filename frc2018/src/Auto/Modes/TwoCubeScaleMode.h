/*
 * TwoCubeScaleMode.h
 *
 *  Created on: Apr 9, 2018
 *      Author: human
 */

#ifndef SRC_AUTO_MODES_TWOCUBESCALEMODE_H_
#define SRC_AUTO_MODES_TWOCUBESCALEMODE_H_

#include "Auto/Modes/AutoMode.h"

class TwoCubeScaleMode  : public AutoMode {
public:
	TwoCubeScaleMode(RobotModel *robot);
	void CreateQueue(string gameData, AutoMode::AutoPositions pos) override;
	void Init();
	virtual ~TwoCubeScaleMode();
};

#endif /* SRC_AUTO_MODES_TWOCUBESCALEMODE_H_ */
