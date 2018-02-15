#ifndef SRC_AUTO_MODES_KOPTESTMPMODE_H_
#define SRC_AUTO_MODES_KOPTESTMPMODE_H_

#include "Auto/Modes/AutoMode.h"

class KOPTestMPMode : public AutoMode {
public:
	KOPTestMPMode(RobotModel *robot);
	void CreateQueue(string gameData, AutoMode::AutoPositions pos);
	void Init();
	virtual ~KOPTestMPMode();
private:
	PathCommand* MPPathCommand_;
};

#endif /* SRC_AUTO_MODES_KOPTESTMPMODE_H_ */
