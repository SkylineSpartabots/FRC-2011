

#include "LineSensors.h"

LineSensors::LineSensors (
		UINT32 leftSensorPort,
		UINT32 middleSensorPort,
		UINT32 rightSensorPort)
{
	m_leftCam	= new DigitalInput(leftSensorPort);
	m_middleCam	= new DigitalInput(middleSensorPort);
	m_rightCam	= new DigitalInput(rightSensorPort);
};


LineSensors::~LineSensors ()
{
	if (m_leftCam)
		delete m_leftCam;
	if (m_middleCam)
		delete m_middleCam;
	if (m_rightCam)
		delete m_rightCam;
};


/****************************************
 * GetLineValue:
 * 	Returns a entry from the LineValue enumeration indicating which cameras
 *	see a line.
 *
 *  Input 	= None
 * Output 	= A value from the LineValue enumeration
 *
 * For autonomous only.
 */
LineSensors::LineValue LineSensors::GetLineValue(void)
{
	int rawValue =	(m_leftCam->Get() ? 1 : 0) &
					(m_middleCam->Get() ? 2 : 0) &
					(m_rightCam->Get() ? 4 : 0);
	
	return (LineSensors::LineValue)(rawValue);
}
