#ifndef SRC_AUTO_MODES_RIGHTSIDETORIGHTSWITCHMODE_H_
#define SRC_AUTO_MODES_RIGHTSIDETORIGHTSWITCHMODE_H_

#include "Auto/Modes/AutoMode.h"

class RightSideToRightSwitchMode : public AutoMode{
public:
	RightSideToRightSwitchMode();
	void CreateQueue(string gameData, AutoMode::AutoPositions pos);
	void Init();
	virtual ~RightSideToRightSwitchMode();
private:
	PathCommand* MPPathCommand_;
};

#endif /* SRC_AUTO_MODES_RIGHTSIDETORIGHTSWITCHMODE_H_ */
