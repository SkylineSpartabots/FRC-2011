

#include "MotorLimitWatchdog.h"

MotorLimitWatchdog::MotorLimitWatchdog(
		const char * watchdogName,
		SpeedController * motor,
		DigitalInput * highLimit,
		DigitalInput * lowLimit):
	Task("MotorLimitWatchDog", (FUNCPTR)MotorLimitWatchdog::TaskWrapper)
{
	m_name = watchdogName;
	m_motor = motor;
	m_highLimit = highLimit;
	m_lowLimit = lowLimit;

	initFieldNames();

	m_statusLogger = new Notifier((TimerEventHandler)MotorLimitWatchdog::LogStatus, this);
	m_statusLogger->StartPeriodic(1.0);

	Task::Start((UINT32)this);
}

MotorLimitWatchdog::~MotorLimitWatchdog()
{
	if (m_statusLogger)
	{
		m_statusLogger->Stop();
		delete m_statusLogger;
	}
	
}


// static wrapper function to callback non-static member function
void
MotorLimitWatchdog::TaskWrapper(void* ThisObject)
{
	// explicit cast pointer to Class pointer
	MotorLimitWatchdog * myself = (MotorLimitWatchdog*) ThisObject;

	// call member function
	myself->Run();
}



/************************************************************************
 * The following should be moved once the bug in
 * SmartDashboard::AnnounceIfNecessary is fixed and replaced with the
 * original method commented out below.
 ***********************************************************************/
void
MotorLimitWatchdog::LogStatus (MotorLimitWatchdog * obj)
{
	SmartDashboard::Log((bool)(obj->m_highLimit->Get()), obj->pHighLimitField);
	SmartDashboard::Log((bool)(obj->m_lowLimit->Get()), obj->pLowLimitField);
	SmartDashboard::Log(obj->m_motor->Get(), obj->pMotorSpeedField);
}



/*
void
MotorLimitWatchdog::LogStatus (MotorLimitWatchdog * obj)
{
	char * pStatusStr = new char[strlen(obj->m_name) + 32];
	char * pStatusText;
	
	pStatusText = strcpy (pStatusStr, obj->m_name);
	pStatusText += strlen(pStatusStr);
	strcat(pStatusText++, " ");
	
	strcpy(pStatusText, "High limit set:");
	SmartDashboard::Log((bool)(obj->m_highLimit->Get()), pStatusStr);

	strcpy(pStatusText, "Ligh limit set:");
	SmartDashboard::Log((bool)(obj->m_lowLimit->Get()), pStatusStr);

	strcpy(pStatusText, "Current motor speed:");
	SmartDashboard::Log(obj->m_motor->Get(), pStatusStr);

	delete pStatusStr;
}
*/



void
MotorLimitWatchdog::Run()
{
	bool bLimitSwitchHit;
	
	// Initialize the limit switch state
	if (m_highLimit->Get() || m_lowLimit->Get())
		bLimitSwitchHit = true;
	else
		bLimitSwitchHit = false;

	// So long as the task is still active, keep looping
	while(this->Verify())
	{
		// While one of the limit switches are pressed hang out here until
		//  one of the limit switches is released or the task ends
		while (bLimitSwitchHit && this->Verify())
		{
			if (!m_highLimit->Get() && !m_lowLimit->Get())
				bLimitSwitchHit = false;
		}

		// While neither of the limit switches is pressed hang out here until
		//  one of the limit switches is pressed or the task ends
		while (!bLimitSwitchHit && this->Verify())
		{
			if (m_highLimit->Get() || m_lowLimit->Get())
			{
				m_motor->Set(0);
				bLimitSwitchHit = true;
			}
		}
	}
}
