/*
 * PivotToCubeCommand.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: Grace
 */

#include <Auto/Commands/PivotToCubeCommand.h>

using namespace std;

PivotToCubeCommand::PivotToCubeCommand(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource, bool isDriveStraightDesired) {
	printf("in beginning of pivot to cube command\n");

	context_ = NULL;
	subscriber_ = NULL;

	robot_ = robot;
	navXSource_ = navXSource;
	talonSource_ = talonSource;
	isDriveStraightDesired_ = isDriveStraightDesired;
	printf("isDriveStraightDesired_: %d", isDriveStraightDesired_);

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	pivotCommand_ = NULL;
	driveStraightCommand_ = NULL;

	isDone_ = false;

	desiredPivotDeltaAngle_ = 0.0;
	desiredDistance_ = 0.0;

	numTimesInkPivotToAngleInit = 0;

	currState_ = kPivotToAngleInit;
	nextState_ = kPivotToAngleInit;

	numTimesInkDriveStraightInit = 0;
	timeStartForVision_ = 0.0;
	timeStartForAlignWithPegCommand_ = 0.0;
	printf("in pivot to cube command constructor\n");
}

void PivotToCubeCommand::RefreshIni() {
	//pivotCommand_->RefreshIni();
}

void PivotToCubeCommand::Init() {
	printf("in pivot to cube command init\n");
//	Profiler profiler(robot_, "Align With Peg Init");

	context_ = new zmq::context_t(1);

	try {
		subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
		subscriber_->connect("tcp://10.18.68.12:5801");	// ports have to be from 5800-5810 to be not blocked by FMS
		int confl = 1;
		subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
		subscriber_->setsockopt(ZMQ_RCVTIMEO, 1000);
		subscriber_->setsockopt(ZMQ_SUBSCRIBE, "MESSAGE", 0);
		printf("TRY CATCH SUCCEEDED IN PIVOTTOCUBECOMMAND INIT\n");
	} catch(const zmq::error_t &exc) {
		printf("TRY CATCH FAILED IN PIVOTTOCUBECOMMAND INIT\n");
		std::cerr << exc.what();
	}

	desiredPivotDeltaAngle_ = 0.0;
	desiredDistance_ = 0.0;
	isDone_ = false;

	currState_ = kPivotToAngleInit;
	nextState_ = kPivotToAngleInit;

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	numTimesInkPivotToAngleInit = 0;
	numTimesInkDriveStraightInit = 0;

	timeStartForAlignWithPegCommand_ = robot_->GetTime();
}

void PivotToCubeCommand::Update(double currTimeSec, double deltaTimeSec) {
	double lastDesiredAngle = desiredPivotDeltaAngle_;
	double lastDesiredDistance = desiredDistance_;
	double diffInAngle;

	switch (currState_) {
		case (kPivotToAngleInit) :
			printf("In kPivotToAngleInit\n");
			fflush(stdout);

			ReadFromJetson();
			SmartDashboard::PutNumber("Vision pivot delta angle", desiredPivotDeltaAngle_);
			SmartDashboard::PutNumber("Vision desired distance", desiredDistance_);

			diffInAngle = fabs(lastDesiredAngle - desiredPivotDeltaAngle_);
			printf("Difference in Angle: %f\n", diffInAngle);
			if (fabs(desiredPivotDeltaAngle_) < 2.0) {
				printf("diff in angle is < 2");
				nextState_ = kPivotToAngleInit;
			} else if (fabs(desiredPivotDeltaAngle_) > 2.0) {
				printf("WE'RE GETTING THERE, vision done at: %f\n", robot_->GetTime() - timeStartForVision_);

				printf("ANGLE FOR PIVOT COMMAND: %f\n", desiredPivotDeltaAngle_);
#ifdef nopivot
				nextState_ = currState_;
#else
				if (pivotCommand_ == NULL) {
					printf("trying to construct one...\n");
					fflush(stdout);
					pivotCommand_ = new PivotCommand(robot_, desiredPivotDeltaAngle_, false, navXSource_);
					printf("constructed new pivot command!\n");
					fflush(stdout);
				} else {
					printf("trying to delete pivot command...\n");
					fflush(stdout);
					delete pivotCommand_;
					printf("deleted pivot command");
					pivotCommand_ = new PivotCommand(robot_, desiredPivotDeltaAngle_, false, navXSource_);
					printf("constructed new pivot command!");
				}

				printf("pivotCommand constructed: %f\n", robot_->GetTime() - timeStartForVision_);
				pivotCommand_->Init();
				printf("pivotCommand inited: %f\n", robot_->GetTime() - timeStartForVision_);
				nextState_ = kPivotToAngleUpdate;
#endif
			} else {
				printf("vision done at: %f\n", robot_->GetTime() - timeStartForVision_);
				printf("ANGLE THAT WAS GOOD NO PIVOT: %f\n", desiredPivotDeltaAngle_);
				if(isDriveStraightDesired_) {
					nextState_ = kDriveStraightInit;
				} else {
					isDone_ = true;
				}
			}
			break;

		case (kPivotToAngleUpdate) :
			if (!pivotCommand_->IsDone()) {
				pivotCommand_->Update(currTimeSec, deltaTimeSec);
				nextState_ = kPivotToAngleUpdate;
			} else {
				ReadFromJetson();
				printf("Final Vision Angle: %f\n", desiredPivotDeltaAngle_);
				printf("Pivot To Angle Is Done\n");
				isDriveStraightDesired_ = true;
				if (isDriveStraightDesired_) {
					nextState_ = kDriveStraightInit;
				} else {
					isDone_ = true;
					printf("AlighWithPeg Done \n");
				}
			}
			break;

		case (kDriveStraightInit) :
			printf("In DriveStraightInit\n");
			ReadFromJetson();

			if (fabs(desiredDistance_) > 0.0) {	//test threshold
				printf("DISTANCE FOR COMMAND: %f\n", desiredDistance_);

				driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, angleOutput_, distanceOutput_,
						robot_, desiredDistance_);
				driveStraightCommand_->Init();
				nextState_ = kDriveStraightUpdate;
			} else {
				isDone_ = true;
				printf("Done with AlignWithPeg \n");
			}
			break;

		case (kDriveStraightUpdate) :
			if (!driveStraightCommand_->IsDone()) {
				driveStraightCommand_->Update(0.0, 0.0); 	// add timer later
				nextState_ = kDriveStraightUpdate;
			} else {
				isDone_ = true;
				// no next state
				printf("Done with AlignWithPeg \n");
			}
			break;
	}
	currState_ = nextState_;

	if (robot_->GetTime() - timeStartForAlignWithPegCommand_ > 3.5) {	// Timeout for AlignWithPegCommand
		isDone_ = true;
	}
}

bool PivotToCubeCommand::IsDone() {
	return isDone_;
}

void PivotToCubeCommand::ReadFromJetson() {
//	Profiler profilerFromJetson(robot_, "ReadFromJetson");
	//printf("__________________><____________________this is actually updating?!?_________\n");
	printf("*************************in front of read from jetson*************************\n");
//	fflush(stdout);

	string contents = s_recv(*subscriber_); //ZMQ_NOBLOCK 1
	printf("setting up string contents from subscriber %s \n", contents.c_str());
	fflush(stdout);

	stringstream ss(contents);
	vector<string> result;
	if (!ss.good()) {
		printf("contents are bad, probably not getting anything!! check ethernet connection\n");
		fflush(stdout);
	}

	while(ss.good()) {
		string substr;
		getline( ss, substr, ' ' );
		result.push_back( substr );
		printf("parsing result from jetson /n %s", substr.c_str());
		fflush(stdout);
	}
	contents = contents.c_str();

	if (!contents.empty()) {
		desiredPivotDeltaAngle_ = stod(result.at(0));
		printf("set desired pivot angle %f /n", desiredPivotDeltaAngle_);
		desiredDistance_ = stod(result.at(2));
		printf("set desired distance %f /n", desiredDistance_);
	} else {
		printf("contents empty");
	}

//	try {
//		printf("top of try catch loop");
//		fflush(stdout);
////		zmq_msg_t msg;
//		string contents = ReceiveMsgNoBlock(*subscriber_); //ZMQ_NOBLOCK 1
//		printf("setting up string contents from subscriber %s \n", contents.c_str());
//		fflush(stdout);
//
//		stringstream ss(contents);
//		vector<string> result;
//
//		while(ss.good()) {
//			string substr;
//			getline( ss, substr, ' ' );
//			result.push_back( substr );
//			printf("parsing result from jetson /n %s", substr.c_str());
//			fflush(stdout);
//		}
//
//		desiredPivotDeltaAngle_ = stod(result.at(0));
//		//desiredDistance_ = stod(result.end()
//		printf("contents from jetson: %s\n", contents.c_str());
//		fflush(stdout);
////	} catch (const std::exception &exc) {
//	} catch (...) {
//		printf("TRY CATCH FAILED IN READFROMJETSON\n");
//		fflush(stdout);
////		std::cerr << exc.what();
//		desiredPivotDeltaAngle_ = 0.0;
//		desiredDistance_ = 0.0;
//	}

	printf("in end of read from jetson\n");
}


std::string PivotToCubeCommand::ReceiveMsgNoBlock(zmq::socket_t & socket) {

    zmq::message_t message;
    socket.recv(&message, 1);

    return std::string(static_cast<char*>(message.data()), message.size());
}


void PivotToCubeCommand::Reset() {
	isDone_ = true;
}

PivotToCubeCommand::~PivotToCubeCommand() {
	Reset();
	printf("IS DONE FROM DECONSTRUCTOR\n");
}


