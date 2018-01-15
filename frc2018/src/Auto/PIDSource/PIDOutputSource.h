#ifndef SRC_AUTO_PIDOUTPUTSOURCE_H_
#define SRC_AUTO_PIDOUTPUTSOURCE_H_

#include "WPILib.h"
#include "RobotModel.h"
#include "Auto/PIDSource/PIDInputSource.h"

class AnglePIDOutput : public frc::PIDOutput {
public:
	/**
	 * Initializes pidOutput_ to 0
	 */
	AnglePIDOutput();
	/**
	 * Sets pidOutput_ to the output from PID loop
	 */
	void PIDWrite(double output);
	/**
	 * @return pidOutput_
	 */
	double GetPIDOutput();
	/**
	 * Destructor
	 */
	virtual ~AnglePIDOutput();
private:
	/**
	 * Output from PID loop
	 */
	double pidOutput_;
};

class DistancePIDOutput : public frc::PIDOutput {
public:
	/**
	 * Initializes pidOutput_ to 0
	 */
	DistancePIDOutput();
	/**
	 * Gets output from PID loop and assigns it to pidOutput_
	 */
	void PIDWrite(double output);
	/**
	 * @return pidOutput_
	 */
	double GetPIDOutput();
	/**
	 * Destructor
	 */
	virtual ~DistancePIDOutput();
private:
	/**
	 * Output from PID loop
	 */
	double pidOutput_;
};

#endif /* SRC_AUTO_PIDOUTPUTSOURCE_H_ */
