#include "RobotModel.h"
#include <string.h>

#ifndef SRC_PROFILER_H_
#define SRC_PROFILER_H_

class Profiler {
public:
	Profiler(RobotModel* robot, std::string header);
	double ReadTime();
	virtual ~Profiler();
private:
	RobotModel* robot_;
	double timeStart_;
	std::string header_;
};

#endif /* SRC_PROFILER_H_ */
