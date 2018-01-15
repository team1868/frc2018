#include <Auto/PIDSource/PIDOutputSource.h>

AnglePIDOutput::AnglePIDOutput() {
	pidOutput_ = 0.0;
}

void AnglePIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

double AnglePIDOutput::GetPIDOutput() {
	return pidOutput_;
}

AnglePIDOutput::~AnglePIDOutput() {

}


DistancePIDOutput::DistancePIDOutput() {
	pidOutput_ = 0.0;
}

void DistancePIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

double DistancePIDOutput::GetPIDOutput() {
	return pidOutput_;
}

DistancePIDOutput::~DistancePIDOutput() {

}
