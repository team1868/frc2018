#ifndef SRC_AUTO_MODES_LEFTSIDETOLEFTSWITCHMODE_H_
#define SRC_AUTO_MODES_LEFTSIDETOLEFTSWITCHMODE_H_

#include "Auto/Modes/AutoMode.h"

class LeftSideToLeftSwitchMode : public AutoMode {
public:
	LeftSideToLeftSwitchMode(RobotModel *robot);
	void CreateQueue(string gameData, AutoMode::AutoPositions pos);
	void Init();
	virtual ~LeftSideToLeftSwitchMode();
private:
	PathCommand* MPPathCommand_;
};

#endif /* SRC_AUTO_MODES_LEFTSIDETOLEFTSWITCHMODE_H_ */
