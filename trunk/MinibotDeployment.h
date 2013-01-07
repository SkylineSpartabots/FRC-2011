#ifndef MINIBOT_DEPLOYMENT_H_
#define MINIBOT_DEPLOYMENT_H_

#include "WPILib.h"
#include "MotorLimitWatchdog.h"

class MinibotDeployment
{
	// Deployment constants
	public:
		static const float MINIBOT_DEPLOY_SPEED = 1.0;

	private:
		SpeedController * m_deployMotor;	// The motor that deploys the minibot
		DigitalInput * m_deployFarLimit;	// The outer limit for minibot deployment
		DigitalInput * m_deployNearLimit;	// The closer limit for minibot.
		MotorLimitWatchdog * m_motorWatchdog;

	public:
		MinibotDeployment (
				UINT32 motorPort,
				UINT32 deployedSwitchPort,
				UINT32 retractedSwitchPort);
		~MinibotDeployment ();
		
		bool deploy();
		bool retract();

};


#endif	// MINIBOT_DEPLOYMENT_H_
