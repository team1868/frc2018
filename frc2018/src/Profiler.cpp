#include <Profiler.h>

Profiler::Profiler(RobotModel* robot, std::string header) {
	robot_ = robot;
	header_ = header;
	timeStart_ = robot->GetTime();  //need robotmodel done
}

Profiler::~Profiler() {
	printf("%s Profiler time end: %f\n", header_.c_str(), robot_->GetTime()-timeStart_); //same as above
}

