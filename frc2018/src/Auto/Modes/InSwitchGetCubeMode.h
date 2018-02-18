/*
 * InSwitchGetCube.h
 *
 *  Created on: Feb 12, 2018
 *      Author: starr
 */

#ifndef SRC_AUTO_MODES_INSWITCHGETCUBEMODE_H_
#define SRC_AUTO_MODES_INSWITCHGETCUBEMODE_H_

#include "Auto/Modes/AutoMode.h"

class InSwitchGetCubeMode : public AutoMode{
public:
	InSwitchGetCubeMode(RobotModel *robot);
	void CreateQueue();
	void Init();
	virtual ~InSwitchGetCubeMode();
};

#endif /* SRC_AUTO_MODES_INSWITCHGETCUBEMODE_H_ */
