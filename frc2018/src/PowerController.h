/*
 * PowerController.h
 *
 *  Created on: Mar 6, 2018
 *      Author: Grace
 */

#ifndef SRC_POWERCONTROLLER_H_
#define SRC_POWERCONTROLLER_H_

#include "RobotModel.h"
#include "DriverStation/ControlBoard.h"
#include "Debugging.h"
#include "Logger.h"
#include <vector>

class PowerController {
	public:
		PowerController(RobotModel *robot, ControlBoard *humanControl);
		void Update(double currTimeSec, double deltaTimeSec);
		bool IsBatteryLow();
		void LimitSingle();
		void PriorityScale();
		void IntakeFocus();
		void CompressorCut();
		void RefreshIni();
		void Reset();

		virtual ~PowerController();
	private:
		RobotModel* robot_;
		ControlBoard* humanControl_;
		double totalCurrent_, totalVoltage_;

		double avgLeftCurr_, avgRightCurr_, avgIntakeCurr_, avgCompCurr_, avgRioCurr_;
		int size; // number of past values to average
		std::vector<double> pastLeftCurr_, pastRightCurr_, pastIntakeCurr_,
			pastCompCurr_, pastRioCurr_;

		double driveCurrentLimit_, intakeCurrentLimit_, totalCurrentLimit_;
		double voltageFloor_, pressureFloor_;

		double driveWeight_, compWeight_, intakeWeight_;

		double GetAverage(std::vector<double> v);
};

#endif /* SRC_POWERCONTROLLER_H_ */
