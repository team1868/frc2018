/*
 * CubeInScaleMode.h
 *
 *  Created on: Mar 3, 2018
 *      Author: human
 */

#ifndef SRC_AUTO_MODES_CUBEINSCALEMODE_H_
#define SRC_AUTO_MODES_CUBEINSCALEMODE_H_

#include "Auto/Modes/AutoMode.h"

class CubeInScaleMode : public AutoMode {
public:
	CubeInScaleMode(RobotModel *robot);
	void CreateQueue(string gameData, AutoMode::AutoPositions pos) override;
	void Init();
	virtual ~CubeInScaleMode();
};

#endif /* SRC_AUTO_MODES_CUBEINSCALEMODE_H_ */
