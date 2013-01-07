
#include "MinibotDeployment.h"


MinibotDeployment::MinibotDeployment (
		UINT32 motorPort,
		UINT32 deployedSwitchPort,
		UINT32 retractedSwitchPort)
{
	m_deployMotor = new Victor(motorPort);
	m_deployFarLimit = new DigitalInput (deployedSwitchPort);
	m_deployNearLimit = new DigitalInput(retractedSwitchPort);

	m_motorWatchdog = new MotorLimitWatchdog(
			"Minibot",
			m_deployMotor,
			m_deployFarLimit,
			m_deployNearLimit);
}



MinibotDeployment::~MinibotDeployment ()
{
	if (m_motorWatchdog)
	{
		m_motorWatchdog->Stop();
		delete m_motorWatchdog;
	}

	if (m_deployMotor)
		delete m_deployMotor;
	
	if (m_deployFarLimit)
		delete m_deployFarLimit;
	
	if (m_deployNearLimit)
		delete m_deployNearLimit;
}



/****************************************************************************
 *	Starts the minibot deployment unless the minibot is already fully
 *  deployed.
 * 
 * Input - 
 * 		none
 * Output -
 * 		Returns true if the minibot is fully deployed.
 * 		Returns false if the minibot is not yet fully deployed.
 ***************************************************************************/
bool
MinibotDeployment::deploy()
{
	bool result;
	
	if (m_deployFarLimit->Get())
	{
		m_deployMotor->Set(0);
		result = true;
	}
	else
	{
		m_deployMotor->Set(MINIBOT_DEPLOY_SPEED);
		result=false;
	}

	return result;
}



/****************************************************************************
 *	Starts the minibot retraction unless the minibot is already fully
 *  retracted.
 * 
 * Input - 
 * 		none
 * Output -
 * 		Returns true if the minibot is fully retracted.
 * 		Returns false if the minibot is not yet fully retracted.
 ***************************************************************************/
bool
MinibotDeployment::retract()
{
	bool result;
	
	if (m_deployNearLimit->Get())
	{
		m_deployMotor->Set(0);
		result = true;
	}
	else
	{
		m_deployMotor->Set(MINIBOT_DEPLOY_SPEED * -1);
		result=false;
	}

	return result;
}
