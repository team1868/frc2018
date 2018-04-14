#ifndef SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_
#define SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_

#include "RobotModel.h"
#include <DriverStation/ControlBoard.h>
#include "Auto/Commands/ElevatorHeightCommand.h"


class SuperstructureController {
public:
	SuperstructureController(RobotModel *myRobot, ControlBoard *myHumanControl);

	void Reset();

	void Update(double currTimeSec, double deltaTimeSec);

	void RefreshIni();

	virtual ~SuperstructureController();

	enum SuperstructureState {
		kInit, kIdle, kRampRelease, kRampRaise
	};

private:
	void HoldCube();

	RobotModel *robot_;
	ControlBoard *humanControl_;

	uint32_t currState_;
	uint32_t nextState_;

	double elevatorUpOutput_;
	double elevatorDownOutput_;
	double elevatorCurrentLimit_;
	double elevatorRamp_;
	double elevatorMaxOutput_;
	double rampOutput_;
	double rampReleaseTime_;
	double rampReleaseDiffTime_;
	bool elevatorMovingCurr_;
	bool elevatorMovingLast_;
	bool elevatorCurrLimitReached_;
	double wristCurrentLimit_;
	double wristOutput_;
	bool wristCurrLimitReached_;
	bool wristMovingCurr_;
	bool wristMovingLast_;
};

#endif /* SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_ */
