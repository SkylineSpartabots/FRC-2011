
#include "AutonomousTask.h"
#include "Timer.h"

AutonomousTask::AutonomousTask(RobotDrive * robotDrive, LiftController * lift, LineSensors * sensors):
	Task("AutonomousTask", (FUNCPTR)AutonomousTask::TaskWrapper)
{
}

AutonomousTask::~AutonomousTask()
{
}


// static wrapper function to callback non-static member function
void
AutonomousTask::TaskWrapper(void* ThisObject)
{
	// explicit cast pointer to Class pointer
	AutonomousTask * myself = (AutonomousTask*) ThisObject;

	// call member function
	myself->Run();
}


bool
AutonomousTask::Stop()
{
	pLift->stop();
	pRobotDrive->StopMotor();
	return Task::Stop();
}



void
AutonomousTask::Run()
{
	// Lift the tube
	pLift->extend(1.0, 4);

	// Follow the line to the end
	FollowLine();

	// move right slightly
	pRobotDrive->MecanumDrive_Cartesian(RIGHT * SPEED_FACTOR, NEUTRAL, NEUTRAL);

	// retract the lift slightly
	pLift->retract(0.2, 0.5);
	
	// move backwards
	pRobotDrive->MecanumDrive_Cartesian(NEUTRAL, BACKWARD * SPEED_FACTOR, NEUTRAL);

	// rotate 180 degrees
	pRobotDrive->MecanumDrive_Cartesian(NEUTRAL, NEUTRAL, BACKWARD);

	// move forward
	pRobotDrive->MecanumDrive_Cartesian(NEUTRAL, FORWARD * SPEED_FACTOR, NEUTRAL);

	return;
}



bool
AutonomousTask::FollowLine()
{
	bool result = true;
	LineSensors::LineValue sensorValue = pSensors->GetLineValue();
	
	while (sensorValue != LineSensors::kAll &&
		   sensorValue != LineSensors::kNone)
	{
		float xDirection;
		float yDirection;
		
		switch (sensorValue)
		{
			case LineSensors::kLeft:
				xDirection = LEFT;
				yDirection = NEUTRAL;
				break;

			case LineSensors::kRight:
				xDirection = RIGHT;
				yDirection = NEUTRAL;
				break;

			case LineSensors::kMiddle:
				xDirection = NEUTRAL;
				yDirection = FORWARD;
				break;

			case LineSensors::kLeftAndMiddle:
				xDirection = LEFT;
				yDirection = FORWARD;
				break;

			case LineSensors::kMiddleAndRight:
				xDirection = RIGHT;
				yDirection = FORWARD;
				break;

			case LineSensors::kAll:
				xDirection = NEUTRAL;
				yDirection = NEUTRAL;
				break;

			case LineSensors::kLeftAndRight:
				xDirection = LEFT;
				yDirection = NEUTRAL;
				break;

			case LineSensors::kNone:
				xDirection = LEFT;
				yDirection = NEUTRAL;
				result = false;
				break;
		}

		pRobotDrive->MecanumDrive_Cartesian(
				xDirection * SPEED_FACTOR,
				yDirection * SPEED_FACTOR,
				NEUTRAL);
	}

	pRobotDrive->MecanumDrive_Cartesian(NEUTRAL, NEUTRAL, NEUTRAL);

	return result;
}
