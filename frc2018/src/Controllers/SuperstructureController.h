#ifndef SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_
#define SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_

#include "RobotModel.h"
#include "DriverStation/ControlBoard.h"


class SuperstructureController {
public:
	SuperstructureController(RobotModel *myRobot, ControlBoard *myHumanControl);

	void Reset();

	void Update(double currTimeSec, double deltaTimeSec);

	void RefreshIni();

	virtual ~SuperstructureController();

	enum SuperstructureState {
		kInit, kIdle, kElevatorToHeight, kRamp
	};

private:
	RobotModel *robot_;
	ControlBoard *humanControl_;

	ElevatorHeightCommand *elevatorHeightCommand_;

	uint32_t currState_;
	uint32_t nextState_;

	double intakeMotorOutput_, outtakeMotorOutput_, elevatorOutput_;
	double desiredElevatorHeight_;
};

#endif /* SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_ */
