/*
 * BlankMode.h
 *
 *  Created on: Jan 12, 2018
 *      Author: Lynn D
 */

#ifndef SRC_AUTO_MODES_BLANKMODE_H_
#define SRC_AUTO_MODES_BLANKMODE_H_

#include "Auto/Modes/AutoMode.h"

class BlankMode : public AutoMode {
public:
	BlankMode();
	void CreateQueue(string gameData, AutoMode::AutoPositions pos) override;
	void Init();
	virtual ~BlankMode();
};

#endif /* SRC_AUTO_MODES_BLANKMODE_H_ */
