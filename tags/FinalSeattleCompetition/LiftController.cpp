
#include "LiftController.h"
#include "math.h"


LiftController::LiftController (
		UINT32 motorPort,
		UINT32 highLimitPort,
		UINT32 lowLimitPort)
{
	m_liftMotor = new Victor(motorPort);
	m_highLimit = new DigitalInput (highLimitPort);
	m_lowLimit = new DigitalInput(lowLimitPort);

	m_currentPreset = PRESET_BOTTOM;
	m_currentHeight = 0.0;

	// Height of the pegs offset by the robot height (in inches).
	m_arrayOfHeights[0] = 0;
	m_arrayOfHeights[1] = 39.0  - ROBOT_HEIGHT;
	m_arrayOfHeights[2] = 77.0  - ROBOT_HEIGHT;
	m_arrayOfHeights[3] = 115.0 - ROBOT_HEIGHT;
	m_arrayOfHeights[4] = 0.0;  	// Zero-terminated just in case.


	m_motorWatchdog = new MotorLimitWatchdog(
			"Lift",
			m_liftMotor,
			m_highLimit,
			m_lowLimit);
}



LiftController::~LiftController ()
{
	if (m_motorWatchdog)
	{
		m_motorWatchdog->Stop();
		delete m_motorWatchdog;
	}

	if (m_liftMotor)
		delete m_liftMotor;
	
	if (m_highLimit)
		delete m_highLimit;
	
	if (m_lowLimit)
		delete m_lowLimit;
}



void
LiftController::initButtons(UINT32 bottom, UINT32 peg1, UINT32 peg2, UINT32 peg3)
{
	m_buttons.bottomButton = bottom;
	m_buttons.peg1Button = peg1;
	m_buttons.peg2Button = peg2;
	m_buttons.peg3Button = peg3;
	
	return;
}



bool
LiftController::isAtTop()
{
	bool result;

	if (m_highLimit->Get())
		result = true;
	else
		result = false;
	
	return (result);
}



bool
LiftController::isAtBottom()
{
	bool result;

	if (m_lowLimit->Get())
		result = true;
	else
		result = false;
	
	return (result);
}



bool
LiftController::stop()
{
	m_liftMotor->Set(0);
	
	return true;
}



/****************************************************************************
 * Extends the lift
 * 
 * Input -
 * 		The speed at which to extend the lift
 * Output - 
 * 		Returns true if the lift is moving up
 * 		Returns false if the lift is already at the top or the speed is out
 * 		of range.
 ***************************************************************************/
bool
LiftController::extend(float speed)
{
	bool result;
	
	if (isAtTop())
	{
		m_liftMotor->Set(0);
		result = false;
	}
	else if (speed >= 0 && speed <= 1.0)
	{
		m_liftMotor->Set(min(speed, MAX_LIFT_SPEED)); 
		result = true;
	}
	else
		result = false;
	
	return result;
}



/****************************************************************************
 * Retracts the lift
 * 
 * Input -
 * 		The speed at which to retract the lift
 * Output - 
 * 		Returns true if the lift is moving down
 * 		Resurns false if the lift is already at the bottom or the speed is
 * 		out of range.
 ***************************************************************************/
bool
LiftController::retract(float speed)
{
	bool result;
	
	if (isAtBottom())
	{
		m_liftMotor->Set(0);
		result = false;
	}
	else if (speed >= 0 && speed <= 1.0)
	{
		m_liftMotor->Set(min(speed, MAX_LIFT_SPEED) * -1); 
		result = true;
	}
	else
		result = false;
	
	return result;
}



bool
LiftController::isPresetSelected(GenericHID * inputDevice)
{
	bool result = true;
	
	// Chose the preset (button input)
	if (inputDevice->GetRawButton(m_buttons.bottomButton))
		m_currentPreset = PRESET_BOTTOM;
	else if (inputDevice->GetRawButton(m_buttons.peg1Button))
		m_currentPreset = PRESET_PEG1;
	else if (inputDevice->GetRawButton(m_buttons.peg2Button))
		m_currentPreset = PRESET_PEG2;
	else if (inputDevice->GetRawButton(m_buttons.peg3Button))
		m_currentPreset = PRESET_PEG3;
	else
		result = false;

	return result;
}



/****************************************
 * moveToPreset:
 * Input = 	Peg number
 * Output = Scissor movement
 * 			0  = Haven't hit the target yet.
 * 			1  = I've hit the target!
 * 			-1 = Something is seriously wrong. (Negative 1)
 * TODO
 * - Test
 * - Calibrate numbers
 */	
TriState
LiftController::moveToPreset()
{
	TriState result;

	// Basic checks: make sure that movement is needed or that the lift
	// hasn't hit the limit switches (it should never, but just in case...)
	float targetHeight = m_arrayOfHeights[m_currentPreset];

	if (targetHeight == m_currentHeight)
		result = Success;
	else
	{
		if (isAtTop() || isAtBottom())
			result = Error;
		else
		{
			// Checking to make sure the motor doesn't spin too much.
			float neededDirection = targetHeight - m_currentHeight;
			float motorTurn;
			if (fabs(neededDirection / TURN_TRANSFORM) > 1.0) {
				// If needed distance exceeds the maximum motor movement...
				motorTurn = (float)(GetSign(neededDirection));
				m_currentHeight += (motorTurn * TURN_TRANSFORM);
			} else {
				// If needed distance falls under the maximum motor movement...
				motorTurn = neededDirection / TURN_TRANSFORM;
				m_currentHeight += neededDirection;
			}
		
			if (motorTurn > 0)
				extend(motorTurn);
			else if (motorTurn < 0)
				retract(fabs(motorTurn));
			else
				stop();
		
			result = (m_currentHeight == targetHeight) ? Success : Nominal;
		}		
	}
	
	return result;
}


TriState
LiftController::moveToPeg(PRESETS preset)
{
	m_currentPreset = preset;
	return moveToPreset();
}



float
LiftController::getCurrentHeight()
{
	return m_currentHeight;
}
