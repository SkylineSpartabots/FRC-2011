#ifndef LINE_SENSORS_H_
#define LINE_SENSORS_H_

#include "WPILib.h"

class LineSensors
{
	private:
		DigitalInput * m_leftCam;			// The cameras for autonomous.
		DigitalInput * m_middleCam;			// Left camera follows the line.
		DigitalInput * m_rightCam;			// Left and right from robot's perspective.

	public:
		/* Represents a bit mask where:
		 * 	1 is true for the left camera
		 *	2 is true for the middle camera
		 * 	4 is true for the right camera
		 * 	plus any combinations of these three values
		 */
		typedef enum
		{
			kNone = 0,
			kLeft = 1,
			kMiddle = 2,
			kLeftAndMiddle = 3,
			kRight = 4,
			kLeftAndRight = 5,
			kMiddleAndRight = 6,
			kAll = 7
		} LineValue;


	public:
		LineSensors (UINT32 leftSensorPort, UINT32 middleSensorPort, UINT32 rightSensorPort);
		~LineSensors ();
		LineValue GetLineValue(void);

};


#endif	// LINE_SENSORS_H_
