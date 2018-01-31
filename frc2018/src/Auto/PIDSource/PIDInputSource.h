#ifndef SRC_AUTO_PIDINPUTSOURCE_H_
#define SRC_AUTO_PIDINPUTSOURCE_H_

#include "RobotModel.h"
#include "WPILib.h"


 /*------------------- NAVX PID SOURCE!! -------------------*/

class NavXPIDSource : public frc::PIDSource {
public:
	/**
	 * Assigns the robot and resets the accumulated yaw
	 * @param RobotModel
	 */
	NavXPIDSource(RobotModel *robot);
	/**
	 * Calculates and returns AccumulatedYaw
	 * @return AccumulatedYaw
	 */
	double PIDGet();
	/**
	 * Updates currYAW, calculates deltaYaw and accumulatedYaw
	 */
	double CalculateAccumulatedYaw();
	/**
	 * Sets AccumulatedYaw and deltaYaw to zero
	 * Updates currYaw and lastYaw
	 */
	void ResetAccumulatedYaw();
	/**
	 * Destructor
	 */
	virtual ~NavXPIDSource();

private:
	double currYaw_, lastYaw_, deltaYaw_, accumulatedYaw_;
	RobotModel *robot_;

};

/*------------------- TALON ENCODER PID SOURCE!! -------------------*/

class TalonEncoderPIDSource : public frc::PIDSource {
public:
	/**
	 * Assigns robot and sets averageTalonDistance to 0
	 * @param RobotModel
	 */
	TalonEncoderPIDSource(RobotModel *robot);
	/**
	 *Gets distance from left and right encoders and sets averageTalonDistance
	 *as average of the two
	 *@return averageTalonDistance_
	 */
	double PIDGet();
	/**
	 * Destructor
	 */
	virtual ~TalonEncoderPIDSource();
private:
	RobotModel *robot_;
	/**
	 * Average distance of left and right encoders
	 */
	double averageTalonDistance_;
};

/*------------------- ELEVATOR ENCODER PID SOURCE!! -------------------*/

class ElevatorEncoderPIDSource : public frc::PIDSource {
public:
	ElevatorEncoderPIDSource(RobotModel *robot_);
	double PIDGet();
	virtual ~ElevatorEncoderPIDSource();

private:
	RobotModel *robot_;
	double elevatorHeight_;
};

#endif /* SRC_AUTO_PIDINPUTSOURCE_H_ */
