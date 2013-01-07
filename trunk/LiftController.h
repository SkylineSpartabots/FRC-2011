#ifndef LIFT_CONTROLLER_H
#define LIFT_CONTROLLER_H

#include "WPILib.h"
#include "TypeDefs.h"
#include "MotorLimitWatchdog.h"

int GetSign(float numberInput);

class LiftController
{
	public:
		typedef enum
		{
			PRESET_BOTTOM,
			PRESET_PEG1,
			PRESET_PEG2,
			PRESET_PEG3
		} PRESETS;

		typedef struct
		{
			UINT32	bottomButton;
			UINT32	peg1Button;
			UINT32	peg2Button;
			UINT32	peg3Button;
		} ButtonHolder;
		
	private:
		SpeedController * m_liftMotor;	// Controls the lift
		DigitalInput * m_highLimit;		// The high limit for the lift
		DigitalInput * m_lowLimit;		// The lower limit for the lift
		MotorLimitWatchdog * m_motorWatchdog;	// background task to watch the motor
		PRESETS	m_currentPreset;		// The currently selected preset
		float	m_currentHeight;		// The current height of the lift in inches
		float   m_arrayOfHeights[5];	// The various heights for the lift.
		ButtonHolder m_buttons;			// Buttons to use for the bottom, peg 1, peg 2 and peg 3 presets
		
	public:
		
		static const float MAX_LIFT_SPEED = 0.5;	// Maximum speed the lift should run
		static const float TURN_TRANSFORM = 0.5;		// Debug value
		// TURN_TRANSFORM:	Transforms the wanted distance to the correct amount
		// 					of motor rotations.
		// 					To use:
		// 					Rotation = Distance / TURN_TRANSFORM;
		// 					Distance = Rotation * TURN_TRANSFORM;
		// 					Partially verified value.
		
		static const float ROBOT_HEIGHT 	= 36.5;		// More accurate.
		// ROBOT_HEIGHT:	Measures from the floor to the height of the scissors
		// 					when fully compressed, in inches.  


	public:
		LiftController (
				UINT32 motorPort,
				UINT32 highLimitPort,
				UINT32 lowLimitPort);
		~LiftController ();

		void initButtons (UINT32 bottom, UINT32 peg1, UINT32 peg2, UINT32 peg3);
		bool isAtTop();
		bool isAtBottom();

		bool stop();
		bool extend(float speed);
		bool extend(float speed, double time);
		bool retract(float speed);
		bool retract(float speed, double time);
		
		bool isPresetSelected(GenericHID * inputDevice);
		TriState moveToPreset();
		TriState moveToPeg(PRESETS preset);
		
		float getCurrentHeight();
};


#endif	// LIFT_CONTROLLER_H
