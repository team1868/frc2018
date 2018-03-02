#include <Auto/Commands/PathCommand.h>

//change all of this
const double WHEELBASE_WIDTH = 22.5/12.0; // mid of wheels, tentative
const double WHEEL_DIAMETER = 6.252/12.0; // in ft
const double TIME_STEP = 0.02; // 20 milliseconds, iter robot
const double MAX_VELOCITY = 17.0; // in m/s CHANGE
const double MAX_ACCELERATION = 4.0; // = 4.0??? m/s^2
const double MAX_JERK = 60.0; // = 60.0??? m/s^3
const int TICKS_PER_REV = 256; // optical encoder

const string TRAJECTORY_FILE_DIR = "../MotionProfiling";


PathCommand::PathCommand(RobotModel *robot, Path path, int traj_length) : AutoCommand() {
	robot_ = robot;
	path_ = path;

	isDone_ = false;

	p1_x_ =  p1_y_ = p1_r_ = 0.0;

	p2_x_ =  p2_y_ = p2_r_ = 0.0;

	p3_x_ =  p3_y_ = p3_r_ = 0.0;

	p4_x_ =  p4_y_ = p4_r_ = 0.0;

	leftTrajectory_ = NULL;
	rightTrajectory_ = NULL;

	trajectoryLength_ = traj_length;
	pointLength_ = 0;

	leftEncoderFollower_ = NULL;
	rightEncoderFollower_ = NULL;

	leftEncoderPosition_ = 0;
	rightEncoderPosition_ = 0;

	leftError = 0.0;
	rightError = 0.0;

	lastLeftDistance_ = 0.0;
	lastRightDistance_ = 0.0;

	initialAngle_ = 0.0;
}

void PathCommand::ReadTrajectory() {
	int leftMoProSz = 0;
	int rightMoProSz = 0;

	const double (*leftMotionProfile)[8] = {NULL};
	const double (*rightMotionProfile)[8] = {NULL};

	switch (path_) {
	case kRightSideToRightSwitch:
		{ThreeToRightSwitchTrajectory* temp = new ThreeToRightSwitchTrajectory();
		leftMoProSz = temp->GetLengthOfLeftMotionProfile();
		leftMotionProfile = temp->GetLeftMotionProfile();
		rightMoProSz = temp->GetLengthOfRightMotionProfile();
		rightMotionProfile = temp->GetRightMotionProfile();}
		break;
	case kLeftSideToLeftSwitch:
		{ZeroToLeftSwitchTrajectory* temp = new ZeroToLeftSwitchTrajectory();
		leftMoProSz = temp->GetLengthOfLeftMotionProfile();
		leftMotionProfile = temp->GetLeftMotionProfile();
		rightMoProSz = temp->GetLengthOfRightMotionProfile();
		rightMotionProfile = temp->GetRightMotionProfile();}
		break;
	case kTestKOP:
		{KOPTestTrajectory* temp = new KOPTestTrajectory();
		leftMoProSz = temp->GetLengthOfLeftMotionProfile();
		leftMotionProfile = temp->GetLeftMotionProfile();
		rightMoProSz = temp->GetLengthOfRightMotionProfile();
		rightMotionProfile = temp->GetRightMotionProfile();}
		break;
	default:
		printf("MOTION PROFILE IS NULL\n");
		break;
	}
	printf("in read stuff\n");
	Segment tempLeft;
	Segment tempRight;

	for (int i = 0; i < leftMoProSz; i++) {
		for (int j = 0; j < 8; j++) {
			//printf("hi again %d %d \n",i,j);
			double leftVal = leftMotionProfile[i][j];
			//printf("left %f\n", leftVal);
			double rightVal = rightMotionProfile[i][j];
			//printf("right %f\n", rightVal);
			if (j == 0) { // dt value
				tempLeft.dt = leftVal;
				tempRight.dt = rightVal;
			} else if (j == 1) { // x val
				tempLeft.x = leftVal;
				tempRight.x = rightVal;
			} else if (j == 2) { // y val
				tempLeft.y = leftVal;
				tempRight.y = rightVal;
			} else if (j == 3) { // position
				tempLeft.position = leftVal;
				tempRight.position = rightVal;
			} else if (j == 4) { // velocity
				tempLeft.velocity = leftVal;
				tempRight.velocity = rightVal;
			} else if (j == 5) { // acceleration
				tempLeft.acceleration =  leftVal;
				tempRight.acceleration = rightVal;
			} else if (j == 6) { // jerk
				tempLeft.jerk = leftVal;
				tempRight.jerk = rightVal;
			} else if (j == 7) { // heading
				tempLeft.heading = leftVal;
				tempRight.heading = rightVal;

				leftTrajectory_[i] = tempLeft;
				rightTrajectory_[i] = tempRight;
			}
		}
	}
}

void PathCommand::Init() {

//	switch (path_) {
//	case kRightSideToRightSwitch:
//		break;
//	case kLeftSideToLeftSwitch:
//		break;
////	case kLeftSwitchToRight:
////		break;
////	case kRightSwitchToLeft:
////		break;
////	case kLeftSwitchToLeftScale:
////		break;
////	case kLeftSwitchToRightScale:
////		break;
////	case kRightSwitchToRightScale:
////		break;
////	case kRightSwitchToLeftScale:
////		break;
//	default:
//		printf("MOTION PROFILE IS NULL\n");
//		break;
//	}

//	TrajectoryCandidate candidate;

	// Arguments:
	// Fit Function:        FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
	// Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
	//                      PATHFINDER_SAMPLES_LOW  (10 000)
	//                      PATHFINDER_SAMPLES_FAST (1 000)
	// Time Step:           0.001 Seconds
	// Max Velocity:        15 m/s
	// Max Acceleration:    10 m/s/s
	// Max Jerk:            60 m/s/s/s


	Segment *trajectory = (Segment*)malloc(sizeof(Segment) * trajectoryLength_);

	leftTrajectory_ = (Segment*)malloc(sizeof(Segment) * trajectoryLength_);
	rightTrajectory_ = (Segment*)malloc(sizeof(Segment) * trajectoryLength_);

	printf("traj length %d", trajectoryLength_);
	ReadTrajectory();
	printf("did this work\n");

	for (int i = 0; i < trajectoryLength_; i++) {
		printf("position: %f\n", leftTrajectory_[i].position);
		printf("velocity: %f\n", leftTrajectory_[i].velocity);
		printf("heading: %f\n", leftTrajectory_[i].heading);
	}

	leftEncoderFollower_ = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	leftEncoderFollower_->last_error = 0;
	leftEncoderFollower_->segment = 0;
	leftEncoderFollower_->finished = 0;

	rightEncoderFollower_ = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	rightEncoderFollower_->last_error = 0;
	rightEncoderFollower_->segment = 0;
	rightEncoderFollower_->finished = 0;


	//  need to add to ini
	double lPFac = robot_->pini_->getf("MOTION PROFILE PID", "lPFac", 1.0);
	double lIFac = robot_->pini_->getf("MOTION PROFILE PID", "lIFac", 0.0);
	double lDFac = robot_->pini_->getf("MOTION PROFILE PID", "lDFac", 0.0);
	double lVFac = robot_->pini_->getf("MOTION PROFILE PID", "lVFac", 1.0);
	double lAFac = robot_->pini_->getf("MOTION PROFILE PID", "lAFac", 0.0);

	double rPFac = robot_->pini_->getf("MOTION PROFILE PID", "rPFac", 1.0);
	double rIFac = robot_->pini_->getf("MOTION PROFILE PID", "rIFac", 0.0);
	double rDFac = robot_->pini_->getf("MOTION PROFILE PID", "rDFac", 0.0);
	double rVFac = robot_->pini_->getf("MOTION PROFILE PID", "rVFac", 1.0);
	double rAFac = robot_->pini_->getf("MOTION PROFILE PID", "rAFac", 0.0);

	leftEncoderPosition_ = robot_->GetLeftEncoderValue();

	leftEncoderConfig_ = { leftEncoderPosition_, TICKS_PER_REV, WHEEL_DIAMETER * M_PI,      // Position, Ticks per Rev, Wheel Circumference
							 lPFac, lIFac, lDFac, lVFac / MAX_VELOCITY, lAFac};          // Kp, Ki, Kd and Kv, Ka

	rightEncoderPosition_ = robot_->GetRightEncoderValue();

	rightEncoderConfig_ = { rightEncoderPosition_, TICKS_PER_REV, WHEEL_DIAMETER * M_PI,
							  rPFac, rIFac, rDFac, rVFac / MAX_VELOCITY, rAFac};

	// To make sure SRX's encoder is updating the RoboRIO fast enough
	// If using different controller for slave, set status frame higher

	robot_->leftMaster_->SetStatusFramePeriod(ctre::phoenix::motorcontrol::Status_3_Quadrature, 10, 100);
	robot_->leftSlave_->SetStatusFramePeriod(ctre::phoenix::motorcontrol::Status_3_Quadrature, 10, 100);
	robot_->rightMaster_->SetStatusFramePeriod(ctre::phoenix::motorcontrol::Status_3_Quadrature, 10, 100);
	robot_->rightSlave_->SetStatusFramePeriod(ctre::phoenix::motorcontrol::Status_3_Quadrature, 10, 100);
	// CHANGE TIMEOUT, LAST PARAM


//	FILE *fp_leftTraj = fopen("/home/lvuser/left_trajectory.csv", "w");
//	pathfinder_serialize_csv(fp_leftTraj, leftTrajectory_, trajectoryLength_);
//	fclose(fp_leftTraj);
//
//	FILE *fp_rightTraj = fopen("/home/lvuser/right_trajectory.csv", "w");
//	pathfinder_serialize_csv(fp_rightTraj, rightTrajectory_, trajectoryLength_);
//	fclose(fp_rightTraj);

	initialAngle_ = robot_->GetNavXYaw();

	printf("AT THE END OF PATH COMMAND INIT\n");

}

void PathCommand::Update(double currTimeSec, double deltaTimeSec) {
	leftEncoderPosition_ = robot_->GetLeftEncoderValue();
	rightEncoderPosition_ = robot_->GetRightEncoderValue();

	leftError = leftEncoderFollower_->last_error;
	rightError = rightEncoderFollower_->last_error;
	SmartDashboard::PutNumber("Left Error", leftError);
	SmartDashboard::PutNumber("Right Error", rightError);

	// Arg 1: The EncoderConfig
	// Arg 2: The EncoderFollower for this side
	// Arg 3: The Trajectory generated from `pathfinder_modify_tank`
	// Arg 4: The Length of the Trajectory (length used in Segment seg[length];)
	// Arg 5: The current value of your encoder

	//printf("IN PATH UPDATE!!!!!\n");
	//printf("trajectory length in update: %i\n", trajectoryLength_);
	//printf("leftEncoderPosition: %i\n", leftEncoderPosition_);
	//printf("rightEncoderPosition: %i\n", rightEncoderPosition_);

	double l = pathfinder_follow_encoder(leftEncoderConfig_, leftEncoderFollower_, leftTrajectory_, trajectoryLength_, leftEncoderPosition_);
	double r = pathfinder_follow_encoder(rightEncoderConfig_, rightEncoderFollower_, rightTrajectory_, trajectoryLength_, rightEncoderPosition_);

	// -- using l and r from the previous code block -- //
	double gyro_heading = robot_->GetNavXYaw() - initialAngle_;

	double desired_heading = r2d(leftEncoderFollower_->heading);
	if (desired_heading >= 180.0) {
		desired_heading -= 360.0;
	}

	double angle_difference = desired_heading - gyro_heading;    // Make sure to bound this from -180 to 180, otherwise you will get super large values
	double turn = 1.0 * (1.0/80.0) * angle_difference;

	SmartDashboard::PutNumber("Left output", l);
	SmartDashboard::PutNumber("Right output", r);

	robot_->SetDriveValues(RobotModel::kLeftWheels, l);
	robot_->SetDriveValues(RobotModel::kRightWheels, r);

//	if (!logData_.is_open()) {
//		logData_.open(Logger::GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_moprolog.csv")).c_str()), std::ofstream::out | std::ofstream::app);
//		logData_ << "Time, DeltaTime, LeftEncoderValue, RightEncoderValue, LeftDistance, RightDistance, LeftExpectedDistance, RightExpectedDistance, LeftVelocity, Right Velocity, "
//				 << "LeftExpectedVelocity, RightExpectedVelocity, LeftError, RightError, LeftOutput, RightOutput, Turn, NavXAngle, ExpectedHeading, ExpectedHeading_Edited" << "\r\n";
//	}
//
//	logData_ << robot_->GetTime() << ", " <<
//			   deltaTimeSec << ", " <<
//			   robot_->GetLeftEncoderValue() << ", " <<
//			   robot_->GetRightEncoderValue() << ", " <<
//			   robot_->GetLeftDistance() << ", " <<
//			   robot_->GetRightDistance() << ", " <<
////			   get_expected_position(leftEncoderFollower_) << ", " <<
////			   get_expected_position(rightEncoderFollower_) << ", " <<
//			   (robot_->GetLeftDistance() - lastLeftDistance_) / deltaTimeSec << ", " <<
//			   (robot_->GetRightDistance() - lastRightDistance_) / deltaTimeSec << ", " <<
//			   leftEncoderFollower_->output << ", " <<
//			   rightEncoderFollower_->output << ", " <<
//			   leftError << ", " <<
//			   rightError << ", " <<
//			   l << ", " <<
//			   r << ", " <<
//			   turn << ", " <<
//			   gyro_heading << ", " <<
//			   r2d(leftEncoderFollower_->heading) << ", " <<
//			   desired_heading << "\r\n";
//
//	logData_.flush();

	lastLeftDistance_ = robot_->GetLeftDistance();
	lastRightDistance_ = robot_->GetRightDistance();
}

bool PathCommand::IsDone() {
	if ((leftEncoderFollower_->finished == 1) && (rightEncoderFollower_->finished == 1)) {
		printf("DONE WITH PATH COMMAND\n");
		robot_->SetDriveValues(RobotModel::kLeftWheels, 0.0);
		robot_->SetDriveValues(RobotModel::kRightWheels, 0.0);

		isDone_ = true;

		// Free memory for the followers and trajectories
		if (leftEncoderFollower_ != NULL) {
			free(leftEncoderFollower_);
			leftEncoderFollower_ = NULL;
		}
		if (rightEncoderFollower_ != NULL) {
			free(rightEncoderFollower_);
			rightEncoderFollower_ = NULL;
		}
		if (leftTrajectory_ != NULL) {
			free(leftTrajectory_);
			leftTrajectory_ = NULL;
		}
		if (rightTrajectory_ != NULL) {
			free(rightTrajectory_);
			rightTrajectory_ = NULL;
		}
//		free(trajectory);
//		free(points);

		return true;
	} else {
		isDone_ = false;
		return false;
	}
}

void PathCommand::Reset() {

}

PathCommand::~PathCommand() {

}
