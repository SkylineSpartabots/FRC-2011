
#ifndef AUTONOMOUS_TASK_H_
#define AUTONOMOUS_TASK_H_

#include "WPILib.h"
#include "LiftController.h"
#include "LineSensors.h"


class AutonomousTask : public Task
{
	static const float DELAY_VALUE 		= 0.01;		// In seconds
	static const float SPEED_FACTOR = 0.5;
	static const float FORWARD = -1.0;
	static const float BACKWARD = 1.0;
	static const float LEFT = -1.0;
	static const float RIGHT = 1.0;
	static const float NEUTRAL = 0.0;

	private:
		Timer timer;					// The only timer
		RobotDrive * pRobotDrive;		// Robot drive system (wheels and whatnot)
		LiftController * pLift;		// Handles the lift
		LineSensors * pSensors;


	public:
		AutonomousTask(RobotDrive * robotDrive, LiftController * lift, LineSensors * sensors);
		~AutonomousTask();
		bool Stop(void);

	private:
		static void TaskWrapper (void *);

		void Run();
		bool FollowLine();
	
};

#endif // AUTONOMOUS_TASK_H_
