
#ifndef MOTOR_LIMIT_WATCHDOG_H_
#define MOTOR_LIMIT_WATCHDOG_H_

#include "WPILib.h"


class MotorLimitWatchdog : public Task
{
	const char * m_name;
	SpeedController * m_motor;
	DigitalInput * m_highLimit;
	DigitalInput * m_lowLimit;
	Notifier * m_statusLogger;

	public:
		MotorLimitWatchdog(
				const char * watchdogName,
				SpeedController * motor,
				DigitalInput * highLimit,
				DigitalInput * lowLimit);

		~MotorLimitWatchdog();

	private:
		static void TaskWrapper (void *);
		static void LogStatus (MotorLimitWatchdog * watchdog);

		void Run();

	/************************************************************************
	 * The following should be moved once the bug in
	 * SmartDashboard::AnnounceIfNecessary is fixed.
	 ***********************************************************************/
	private:
		void initFieldNames()
		{
			pHighLimitField = new char[strlen(m_name) + 20];
			strcpy (pHighLimitField, m_name);
			strcat (pHighLimitField, " High limit set:");

			pLowLimitField = new char[strlen(m_name) + 20];
			strcpy (pLowLimitField, m_name);
			strcat (pLowLimitField, " Ligh limit set:");

			pMotorSpeedField = new char[strlen(m_name) + 25];
			strcpy (pMotorSpeedField, m_name);
			strcat (pMotorSpeedField, " Current motor speed:");
		};
		char * pHighLimitField;
		char * pLowLimitField;
		char * pMotorSpeedField;

};

#endif // MOTOR_LIMIT_WATCHDOG_H_
