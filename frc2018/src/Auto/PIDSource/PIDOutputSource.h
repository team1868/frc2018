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

/**
 * PivotPIDTalonOutput a constructor Class for WPI PIDOutput for Pivoting
 */
class PivotPIDTalonOutput : public frc::PIDOutput {

public:

	/**
	 * PivotPIDTalonOutput is a constructor that initializes robot_ and output_
	 * @param robot a RobotModel
	 */
	PivotPIDTalonOutput(RobotModel *robot);

	/**
	 * PIDWrite a function that initializes output_ and sets the drive values
	 */
	void PIDWrite(double output);

	/**
	 * PivotPIDTalonOutput is a destructor
	 */
	virtual ~PivotPIDTalonOutput();

	/**
	 * @return output
	 */
	double GetOutput();

private:
	RobotModel *robot_;
	double output_;
};

#endif /* SRC_AUTO_PIDOUTPUTSOURCE_H_ */
