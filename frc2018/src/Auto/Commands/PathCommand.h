#ifndef SRC_AUTO_COMMANDS_PATHCOMMAND_H_
#define SRC_AUTO_COMMANDS_PATHCOMMAND_H_

#include "WPILib.h"
#include "Auto/Commands/AutoCommand.h"
#include "Auto/MotionProfiling/MotionProfile.h"
#include "RobotModel.h"
#include <fstream>
#include <string>
#include "Logger.h"
#include "../../../ext/pathfinder/pathfinder.h"

class PathCommand : public AutoCommand {
public:
	enum Path { kLeftSwitchToRight, kRightSwitchToLeft, kLeftSwitchToLeftScale, kLeftSwitchToRightScale, kRightSwitchToRightScale, kRightSwitchToLeftScale };

	/**
	 * Constructor that generates a PathCommand
	 * @param robot a RobotModel
	 * @param Path a path of waypoints generated by motion profile
	 */
	PathCommand(RobotModel *robot, Path path);
	~PathCommand();

	/**
	 * Sets waypoints given by the ini file according to the path and generates the path using the given waypoints. PID Values are received from the ini.
	 * Encoders and navX are "zeroed" and the trajectories for right and left are put into csv files.
	 */
	void Init();
	/**
	 * Reads the encoder values and navX values to send and receive the error from the trajectory so that the robot can correct itself as it follows the path.
	 * The error and data are all put into the log.
	 * @param the current time
	 * @param the change in time since the command started
	 */
	void Update(double currTimeSec, double deltaTimeSec);
	/**
	 * Finishes the task if the command goes through the points in the trajectory and frees the memory storing the trajectory and encoder follower.
	 * @return isDone_
	 */
	bool IsDone();

	double leftError;
	double rightError;

private:
	RobotModel *robot_;
	Path path_;

	// Trajectory points
	float p1_x_, p1_y_, p1_r_;
	float p2_x_, p2_y_, p2_r_;
	float p3_x_, p3_y_, p3_r_;
	float p4_x_, p4_y_, p4_r_;

	Segment *leftTrajectory_, *rightTrajectory_;

	/**
	 * The length of the array for the trajectories
	 */
	int trajectoryLength_;

	EncoderFollower *leftEncoderFollower_, *rightEncoderFollower_;
	EncoderConfig leftEncoderConfig_, rightEncoderConfig_;

	/**
	 * The number of waypoints for the trajectory
	 */
	int pointLength_;
	bool isDone_;

	/**
	 * The position of the left encoder in relation to the initial position at the
	 * start of the command
	 */
	int leftEncoderPosition_;
	/**
	 * The position of the right encoder in relation to the initial position at the
	 * start of the command
	 */
	int rightEncoderPosition_;

	/**
	 * The last point of the left trajectory
	 */
	double lastLeftDistance_;
	/**
	 * The last point of the right trajectory
	 */
	double lastRightDistance_;

	double initialAngle_;

	std::ofstream logData_;
};

#endif /* SRC_AUTO_COMMANDS_PATHCOMMAND_H_ */