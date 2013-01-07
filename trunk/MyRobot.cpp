/*************************************************
 * Skyline High School Robotics Team
 * Spartabots, Team 2976
 * Main code for team robot
 * FRC Robotics competition 2011 - Logomotion
 * 
 * This code is based on the SimpleRobot demo and the code used from last 
 * year's competition.
 * The movement code should be conceptually similar to last year's.
 * Autonomous() and OperatorControl() methods are automatically started
 * by either the driver station or by the field controls. 
 ************************************************/

/*-------------------- Recommended Maximum Length of Lines -------------------*/

#include "WPILib.h"
#include "TypeDefs.h"
#include "LineSensors.h"
#include "MinibotDeployment.h"
#include "LiftController.h"
#include "AutonomousTask.h"
#include "math.h"

/****************************************
 * GetSign:
 * Input  = a float
 * Output = if number is positive, returns 1
 * 			if number is negative, returns -1
 * 			if number equals zero, returns 0
 */
int GetSign(float numberInput)
{
	return ((int)(numberInput > 0.0)) - ((int)(numberInput < 0.0));
}

class MainRobot : public SimpleRobot {
	RobotDrive robotDrive;			// Robot drive system (wheels and whatnot)
	Joystick *stick1;				// Directional control
	Joystick *stick2;				// Lifting control
	Timer timer;					// The only timer

	LineSensors * lineSensors;		// The LineSensors object
	MinibotDeployment * minibot;	// Handles Minibot Deployment
	LiftController *	lift;		// Handles the lift
	
	bool isFastSpeedOn;			// Normal or fast speed?
	bool isSafetyModeOn;		// Safety switch.
	bool isLiftHigh;			// Safety switch, can automatically disable.
	bool isDoingPreset;			// Is the lift moving automatically?
	
	// Port (PWM) assignments
	static const UINT32 LEFT_FRONT_MOTOR_PORT	= kPWMPort_1;
	static const UINT32 LEFT_REAR_MOTOR_PORT	= kPWMPort_2;
	static const UINT32 RIGHT_FRONT_MOTOR_PORT	= kPWMPort_3;
	static const UINT32 RIGHT_REAR_MOTOR_PORT	= kPWMPort_4;
	static const UINT32 LIFT_MOTOR_PORT			= kPWMPort_5;
	static const UINT32 MINIBOT_DEPLOY_PORT		= kPWMPort_6;
	
	// Digital IO assignments
	static const UINT32 LOW_LIFT_DIO			= kDigitalIO_1;
	static const UINT32 HIGH_LIFT_DIO			= kDigitalIO_2;
	static const UINT32 MINIBOT_DEPLOYED_DIO	= kDigitalIO_3;
	static const UINT32 MINIBOT_RETRACTED_DIO	= kDigitalIO_4;
	static const UINT32 LEFT_CAMERA_PORT		= kDigitalIO_7;
	static const UINT32 MIDDLE_CAMERA_PORT		= kDigitalIO_8;
	static const UINT32 RIGHT_CAMERA_PORT		= kDigitalIO_9;
	
	// Joystick assignments
	static const UINT32 MOVE_JOYSTICK_USB		= kUSBPort_1;
	static const UINT32 LIFT_JOYSTICK_USB		= kUSBPort_2;
	
	// Button assignments (Both)
	static const UINT32 ENABLE_SAFETY_BUTTON = kJSButton_6;
	static const UINT32 DISABLE_SAFETY_BUTTON = kJSButton_7;
	
	// Button assignments (Driving)
	static const UINT32 MOVE_FAST_BUTTON = kJSButton_1;
	static const UINT32 ROTATE_RIGHT_BUTTON = kJSButton_3; // Clockwise
	static const UINT32 ROTATE_LEFT_BUTTON = kJSButton_4;  // Counter-clockwise
	static const UINT32 EXTEND_MINIBOT_BUTTON = kJSButton_11;
	static const UINT32 RETRACT_MINIBOT_BUTTON = kJSButton_10;
	// ROTATE_RIGHT_BUTTON:	The center button, rotates clockwise.
	// ROTATE_LEFT_BUTTON:	The left button, rotates counter-clockwise.
	// The counterclockwise button was mapped to the center button because
	// mapping it to the right button would force the thumb to move too much.
	
	// General constants
	static const float GAMEPLAY_TIME 	= 120.0;
	static const float SPEED_DECREASE 	= 0.5;
	static const float DELAY_VALUE 		= 0.01;		// In seconds
	// GAMEPLAY_TIME: 	How long teleoperated mode lasts (in seconds)
	// SPEED_DECREASE:	The factor by which the speed should decrease in normal
	// 					mode.  Multiply the output, not divide.
	// DELAY VALUE:		How long the robot should wait in movement loops.
	//					Motors have a minimum update speed.
	
	// Autonomous constants
	static const float FAST_AUTO_TIME = 10.0;
	static const float AUTO_CORRECTION  = 0.1;
	static const int MAX_NO_LINE = 50;				// Needs calibration
	static const LiftController::PRESETS TARGET_PEG_AUTO = LiftController::PRESET_PEG3;
	// FAST_AUTO_TIME:	The time in seconds the robot is allowed to drive at
	// 					maximum speed.  The robot must eventually slow down
	// 					to avoid running into a pole
	// AUTO_CORRECTION:	The value at which the robot will attempt correcting
	// 					itself when it diverges from the line.
	// MAX_NO_LINE:		How many iterations the robot can go without detecting
	// 					a line before shutting down (gone rogue)
	// TARGET_PEG_AUTO: The chosen target peg.  May have to be turned into a 
	// 					variable if switches are incorporated.
	
	// Lift contants
	static const float SAFETY_HEIGHT = 77.0;		// Probably inaccurate
	static const float TURN_TRANSFORM = -0.5;		// Debug value
	// SAFETY_HEIGHT:	When the lift exceeds this height (in inches), the 
	//					robot is deemed too top-heavy to move at high speeds.
	// TURN_TRANSFORM:	Transforms the wanted distance to the correct amount
	// 					of motor rotations.
	// 					To use:
	// 					Rotation = Distance / TURN_TRANSFORM;
	// 					Distance = Rotation * TURN_TRANSFORM;
	// 					Partially verified value.
	
		
public:
	/****************************************
	 * MainRobot: (The constructor)
	 * Mandatory method.
	 * TODO:
	 * - Tweak anything related to the scissor lift - verify values.
	 * - Find out how to configure Victor.
	 */
	MainRobot(void):
		// Below: The constructor needs to know which port connects to which
		// motor so it can control the Jaguars correctly.
		// See constants at top.
		robotDrive(LEFT_FRONT_MOTOR_PORT, LEFT_REAR_MOTOR_PORT, 
		RIGHT_FRONT_MOTOR_PORT, RIGHT_REAR_MOTOR_PORT)
		{
			SmartDashboard::init();
			GetWatchdog().SetExpiration(0.1);  				// In seconds.
			stick1 = new Joystick(MOVE_JOYSTICK_USB); 		// Joystick 1
			stick2 = new Joystick(LIFT_JOYSTICK_USB);		// Joystick 2
			
			minibot = new MinibotDeployment (
					MINIBOT_DEPLOY_PORT,
					MINIBOT_DEPLOYED_DIO,
					MINIBOT_RETRACTED_DIO);

			lineSensors = new LineSensors (
					LEFT_CAMERA_PORT,
					MIDDLE_CAMERA_PORT,
					RIGHT_CAMERA_PORT);
			
			lift = new LiftController (
					LIFT_MOTOR_PORT,
					HIGH_LIFT_DIO,
					LOW_LIFT_DIO);
			lift->initButtons(
					kJSButton_2,	// Botton top button
					kJSButton_4,	// Left top button
					kJSButton_3, 	// Center top button
					kJSButton_5); 	// Right top button

			
			// The wiring was inverted on the left motors, so the below
			// is necessary.
			robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
			robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
			
			isFastSpeedOn = false;
			isSafetyModeOn = true;
			isLiftHigh = false;
			// isSafetyModeOn:  Controlled by the driver -- should only have to
			// 					choose once.
			// isLiftHigh: 		Controlled by the program -- turns true only 
			//					when height is too high, otherwise turns false.
			
			isDoingPreset = false;
			
			GetWatchdog().SetEnabled(true);
			UpdateDashboard("TestingTestingTestingTesting"
							"TestingTestingTestingTestingTesting");
			UpdateDashboard("Finished initializing.");
		}
	
	
	
	
	/****************************************
	 * Autonomous:
	 * Input = Data from driver station or field.
	 * Output = Robot movement (hanging ubertubes)
	 * Mandatory method.
	 * Line tracks edge.  Line should be under left camera only.
	 * If line detected by middle camera, adjust.
	 * Dead reckoning used.
	 * TODO:
	 * - Calibrate numbers and code
	 * - Test
	 * - Run through the flow of logic (double-checking)
	 */
	void Autonomous(void)
	{
		AutonomousTask * pTask = new AutonomousTask (&robotDrive, lift, lineSensors);

		pTask->Start((UINT32)pTask, (UINT32)this);

		while (IsAutonomous() && IsEnabled())
		{
			GetWatchdog().Feed();
			Wait(.01);
			
		}
		pTask->Stop();
		
/********************************************************************
		// Part 0 - initialization.
		//GetWatchdog().SetEnabled(false);
		timer.Reset();
		timer.Start();
		
		// Alternate hardcoded alternative.
		for(int i=0; i<6; i++) {
			// Pick the tube off the ground.
			lift->extend(1.0);
			if(IsAutoDone()) {
				return;
			}
			FatalityChecks(stick1, stick2);
			Wait(DELAY_VALUE);
		}
		lift->stop();
		
		FatalityChecks(stick1, stick2);
		
		for(int i=0; i<26; i++) {
			// Move closer.
			robotDrive.HolonomicDrive(0.5, 0, 0);
			if(IsAutoDone()) {
				return;
			}
		
			FatalityChecks(stick1, stick2);
			Wait(DELAY_VALUE);
		}
		robotDrive.HolonomicDrive(0, 0, 0);
		for(int i=0; i<11; i++) {
			// Move lift up more.
			lift->extend(1.0);
			if(IsAutoDone()) {
				return;
			}
		
			FatalityChecks(stick1, stick2);
			Wait(DELAY_VALUE);
		}
		lift->stop();
		
		// Wiggle.
		robotDrive.HolonomicDrive(0.5, 0, 0);
		
		FatalityChecks(stick1, stick2);
		robotDrive.HolonomicDrive(0, 0, 0);
		Wait(DELAY_VALUE);
		
		if(IsAutoDone()) {
			return;
		}
		
		FatalityChecks(stick1, stick2);
		lift->retract(0.2);
		
		FatalityChecks(stick1, stick2);
		lift->stop();
		
		FatalityChecks(stick1, stick2);
		for(int i=0; i<5; i++) {
			robotDrive.HolonomicDrive(-.5, 0, 0);
			if(IsAutoDone()) {
				return;
			}
		
			FatalityChecks(stick1, stick2);
			Wait(DELAY_VALUE);
		}
		
		FatalityChecks(stick1, stick2);
		// Retract lift.
		for(int i=0; i<20; i++) {
			lift->retract(1.0);
			if(IsAutoDone()) {
				return;
			}
		
			FatalityChecks(stick1, stick2);
			Wait(DELAY_VALUE);
		}
		
		FatalityChecks(stick1, stick2);
		// Wait for everything to end.
		while(IsAutoDone()) {
		
			FatalityChecks(stick1, stick2);
			Wait(DELAY_VALUE);
		}
*****************************************************/
		
		/* Proposed snip.
		
		// If no line is detected, increments this.  If too high, robot stops.
		bool isAtEnd = false;
		bool isLiftDone = false;
		bool isError = false;
		const char *errorMessage = " ";
		UpdateDashboard("0: Starting Autonomous.");
		float emergencyRotate = 0.3;
		
		// Part 1 - Following the line.
		while(IsAutonomous() && !isAtEnd)
		{
			float rotation = 0.0;
			float magnitude = 5.0;
			float direction = 0.0;
			int safetyCount = 0;
			emergencyRotate = -emergencyRotate;
			
			switch (lineSensors->GetLineValue())
			{
				case LineSensors::kNone:
					// Nothing - going rogue.
					safetyCount++;
					rotation = emergencyRotate;
				case LineSensors::kLeft:
				case LineSensors::kMiddle:
				case LineSensors::kLeftAndMiddle:
				case LineSensors::kRight:
				case LineSensors::kLeftAndRight:
				case LineSensors::kMiddleAndRight:
					rotation = 0.0;
					safetyCount = 0;
					break;
				case LineSensors::kAll:
					safetyCount = 0;
					isAtEnd = true;
					break;
				default:
					// Are there more motors?
					isError = true;
					errorMessage = "ERROR: a1 - Too many line detectors?";
			}
			
			if (!isAtEnd)
			{
				if (safetyCount == MAX_NO_LINE) {
					isError = true;
					errorMessage = "ERROR: a1 - Drifted too far.";
				}
				
				// Magic here - see method DriveHost for more info.
				robotDrive.HolonomicDrive(magnitude, direction, rotation);
				if (!isLiftDone)
					isError = AutoLift(TARGET_PEG_AUTO, isLiftDone);
				
				// Error-catching and checks
				if (isError)
					break;
				if (IsAutoDone()) {
					// Autonomous won't automatically quit when
					// it's operator control.
					return;
				}
				GetWatchdog().Feed();
				FatalityChecks(stick1, stick2);
				UpdateDashboard("1: Following the line...");
				Wait(DELAY_VALUE);
			}
		}
		
		
		// Part two - if at the end, try raising the lift.
		if (isAtEnd) {
			for(int i = 0; i<25; i++) {
				//if (!isLiftDone)
				//	isError = AutoLift(TARGET_PEG_AUTO, isLiftDone);
				
				lift->extend(0.3);
				
				// Error-catching and checks
				if (isError) {
					errorMessage = "ERROR: a2 - Lift hit higher limit switch?";
					break;
				}
				if (IsAutoDone())
					return;
				GetWatchdog().Feed();
				FatalityChecks(stick1, stick2);
				UpdateDashboard("2: End of the line...");
				Wait(DELAY_VALUE);
			}
			lift->stop();
			
			// Part 3 - hardcoded wiggle bit.
			robotDrive.HolonomicDrive(0.5, 0, 0);
			lift->retract(0.1);
			robotDrive.HolonomicDrive(-0.5, 0, 0);
			
			// Part 3 - attempt lowering the lift.
			isLiftDone = false;
			while (!isLiftDone) {
				if (!isLiftDone)
					isError = AutoLift(LiftController::PRESET_BOTTOM, isLiftDone);
				
				// Error-catching and checks
				if (isError) {
					errorMessage = "ERROR: a3 - Lift hit lower limit switch?";
					break;
				}
				if (IsAutoDone())
					return;
				GetWatchdog().Feed();
				FatalityChecks(stick1, stick2);
				UpdateDashboard("3: Tucking lift away...");
				Wait(DELAY_VALUE);
			}
		}
		
		// Part 4 - If any time left, wait here.  If errors, default to here.
		while(IsAutoDone()) {
			GetWatchdog().Feed();
			FatalityChecks(stick1, stick2);
			UpdateDashboard((isError) ? errorMessage : "4: Autonomous finished.");
			Wait(DELAY_VALUE);
		}
		
		*/
	}
	
	
	
	
	/****************************************
	 * OperatorControl:
	 * Input = Data from driver station or field
	 * Output = Robot movements
	 * Mandatory method. 
	 * TODO:
	 * None
	 */
	void OperatorControl(void)
	{
		GetWatchdog().SetEnabled(true);

		timer.Reset();
		timer.Start();
		GetWatchdog().Feed();
		isFastSpeedOn = false;
		isSafetyModeOn = false;
		bool isLiftGood;
		UpdateDashboard("Starting Operator Control");
		while(IsOperatorControl()) {
			FatalityChecks(stick1, stick2);
			DriveHost(stick1);
			isLiftGood = ManualLift(stick2);
			MinibotDeploy(stick1);
			
			GetWatchdog().Feed();
			UpdateDashboard(isLiftGood ? " " : "Scissor Error");
			Wait(DELAY_VALUE);
		}
	}
	
	
	
	
	/****************************************
	 * FatalityChecks:
	 * Input = Both joysticks, error codes from ManualLift
	 * Output = None
	 * Handles 
	 * - Joystick disconnects
	 * - Toggling safety mode
	 * TODO:
	 * - Find out how 'wpi_fatal' works
	 * - Replace NULL with '0'? (zero)
	 */
	void FatalityChecks(GenericHID *moveStick, GenericHID *liftStick)
	{
		// Terminate if a joystick is disconnected.
		bool badMove = (NULL == moveStick);
		bool badLift = (NULL == liftStick);
		if (badMove || badLift) {
			if (badMove && !badLift) {
				UpdateDashboard("ERROR: Stick 1 disconnected");
			} else if (!badMove && badLift) {
				UpdateDashboard("ERROR: Stick 2 disconnected");
			} else if (badMove && badLift) {
				UpdateDashboard("ERROR: Stick 1 and stick 2 disconnected");
			}
			wpi_fatal(NullParameter);
			return;
		}

		if (false == GetWatchdog().IsAlive()) {
			UpdateDashboard("ERROR: The watchdog died");
			GetWatchdog().Kill();
			wpi_fatal(NullParameter);
			return;
		}
		
		if (moveStick->GetRawButton(ENABLE_SAFETY_BUTTON) ||
			liftStick->GetRawButton(ENABLE_SAFETY_BUTTON)) {
			isSafetyModeOn = true;
		}
		if (moveStick->GetRawButton(DISABLE_SAFETY_BUTTON) ||
			liftStick->GetRawButton(DISABLE_SAFETY_BUTTON)) {
			isSafetyModeOn = false;
		}
		
		// If the scissor-lift is too high, it might topple at higher speeds.
		isLiftHigh = lift->isAtTop();
	}
	
		

	
	/****************************************
	 * DriveHost:
	 * Input = Joystick data
	 * Output = Robot movement (controls mechanum wheels)
	 * Radically altered code from last year.
	 * Altered so it uses the new buttons.
	 * TODO:
	 * - Find out how to pass robotDrive here without breaking anything
	 *   (Not a necessary task).
	 */
	void DriveHost(GenericHID *moveStick)
	{		
		// Safety primarily to prevent toppling or for safer demos.
		isFastSpeedOn = false;
		if (moveStick->GetRawButton(MOVE_FAST_BUTTON)) {
			//if ((false == isSafetyModeOn) && (false == isLiftHigh)) {
			isFastSpeedOn = true;
			//}
		}	
		
		// Magnitude: [-1.0 to 1.0] - How far to travel.
		// Direction: In degrees	- Which way to travel.
		// Rotation : [-1.0 to 1.0] - How much to turn.
		// Joystick returns a float in range [-1.0 to 1.0] automatically.
		// Using moveStick will not compile.
		float magnitude = fabs(stick1->GetMagnitude());	// fabs = Float abs
		float direction = stick1->GetDirectionDegrees();
		float rotationSpeed = (moveStick->GetThrottle() - 1.1) * -0.5 + 0.07;
		float rotationPress = int(moveStick->GetRawButton(ROTATE_RIGHT_BUTTON)) 
							  - int(moveStick->GetRawButton(ROTATE_LEFT_BUTTON));
		float rotation = rotationSpeed * rotationPress;
		
		direction = (direction < 0.0) ? direction + 360.0 : direction;
		
		// Just in case - prevents values from being over 1.0 (absolute value)
		// Higher numbers cause motors to spin alarmingly fast.
		magnitude = (magnitude > 1.0) ?  1.0 : magnitude;
		rotation  = (rotation > 1.0)  ?  1.0 : rotation;
		rotation  = (rotation < -1.0) ? -1.0 : rotation;
		
		if (!isFastSpeedOn) {
			magnitude *= SPEED_DECREASE;
			rotation  *= SPEED_DECREASE;
		}
		
		// Prevents drift if values are too close to zero.
		magnitude = (magnitude < 0.1) ? 0.0 : magnitude;
		rotation  = (fabs(rotation) < 0.04) ? 0.0 : rotation;
		
		// This is where the magic happens.
		robotDrive.HolonomicDrive(magnitude, direction, rotation);
		
		SmartDashboard::Log(direction, "JS- Distance: ");
		SmartDashboard::Log(magnitude, "JS- Magnitude: ");
		SmartDashboard::Log(rotation, "JS- Rotation: ");
	}
	
	
	
	
	/****************************************
	 * ManualLift:
	 * Input = 	The motor for the lift (Victors only(?))
	 * 			Data from Joystick 2
	 * Output = Scissor lift movement
	 * 			false = Error of some kind (probably passed from ScissorPreset)
	 * 			true  = Everything is just dandy.
	 * TURN_TRANSFORM- Distance * TURN_TRANSFORMS == amount of motor turns
	 * Victor turns at rate of [-1.0 to 1.0]
	 * 
	 * TODO:
	 * - Test code
	 * - Test MAXIMUM_TURN, calibrate numbers
	 * - Use limit switches for max or min, not dead reckoning.
	 */
	bool ManualLift(GenericHID *liftStick)
	{
		bool result;
		
		isDoingPreset = lift->isPresetSelected (liftStick);
		
		// If presets were not overriden, continue moving.
		if (isDoingPreset)
		{
			int valueReturned = lift->moveToPreset();
			if (1 == valueReturned) {
				isDoingPreset = false;
			} else if (-1 == valueReturned){
				result = false;	// Error
			}
		}
		else
		{
			// Pushing joystick up returns a negative Y-value, oddly.
			float userInput = -liftStick->GetY();

			SmartDashboard::Log(userInput, "absoluteInput: ");
			
			userInput = (fabs(userInput) > 1.0) ? GetSign(userInput) : userInput;
			
			// Override any presets.
			isDoingPreset = false;
			
			// Positive = counter-clockwise spin (when facing the motor)
			// Counter-clockwise = rise.
			// Luckily, a positive value corresponds with a rise in height(?).
			if (userInput > 0.05)
				result = lift->retract(fabs(userInput));
			else if (userInput < -0.05)
				result = lift->extend(fabs(userInput));
			else
				result = lift->stop();
			SmartDashboard::Log(userInput, "userInput: ");
		}		
	
		return result;
	}
	
	
	
	
	/****************************************
	 * AutoLift:
	 * Input 	= Target peg number
	 * 			  Pointer to bool value (if finished)
	 * 			  Bool error value (true returned if no error)
	 * Output 	= Scissor-lift movement
	 * 			  Changes bool value that was passed.
	 * 			  Returns true if no error, false if error.
	 * For autonomous only.
	 */
	bool AutoLift(LiftController::PRESETS targetPeg, bool &isFinished) 
	{
		int liftOutput = lift->moveToPeg(targetPeg);
		isFinished = (1 == liftOutput) ? true : false;
		return (-1 == liftOutput) ? false : true;
	}
	
	
	
	
	/****************************************
	 * Minibot Deployer
	 * Input =	Deployment motor
	 * 			Button push
	 * 			Further limit
	 * 			Closer limit
	 * Output = Minibot deploys
	 * TODO:
	 * - Write a better one
	 * - Find accurate motor numbers
	 * - Add delays to prevent motor from breaking?
	 */
	void MinibotDeploy(GenericHID *moveStick)
	{
		if (moveStick->GetRawButton(EXTEND_MINIBOT_BUTTON))
			minibot->deploy();
		else if (moveStick->GetRawButton(RETRACT_MINIBOT_BUTTON))
			minibot->retract();
	}
	
	
	
	
	/****************************************
	 * IsAutoDone:
	 * Input 	= None
	 * Output 	= True if no longer autonomous
	 * For autonomous only.
	 * This checks to see if autonomous is over.
	 * In case the the competition-broadcast-thing fails and doesn't
	 * send the single stating that Autonomous is over, this also
	 * provides some interrupts.
	 * Autonomous can be ended by turning safety off or by
	 * pushing a joystick nearly to the max (any direction)
	 * while holding the trigger down.
	 */
	bool IsAutoDone(void)
	{
		// Can disable autonomous by turning safety off.
		bool safetyKill = 	stick1->GetRawButton(DISABLE_SAFETY_BUTTON) ||
							stick2->GetRawButton(DISABLE_SAFETY_BUTTON);
		
		// Can disable autonomous by attempting to move at max speed.
		bool moveKill = 	(fabs(stick1->GetMagnitude()) > 0.9) &&
							(stick1->GetRawButton(MOVE_FAST_BUTTON));
		bool liftKill =		(fabs(stick2->GetMagnitude()) > 0.9) &&
							(stick2->GetRawButton(MOVE_FAST_BUTTON));
		
		// Disable if a signal is sent out by the driver station.
		bool systemKill = (false == IsAutonomous()) || IsOperatorControl();
		
		return safetyKill || moveKill || liftKill || systemKill;
	}

	
	
	
	/****************************************
	 * UpdateDashboard:
	 * Input = none
	 * Output = Safety mode
	 * 			Watchdog state
	 * 			Robot Speed
	 * 			System state (Autonomous or Teleoperated?)
	 * 			Robot state (Enabled or Disabled?)
	 * 			Timer
	 * 			Minibot alert
	 * Dependent on the 'SmartDashboard' class from the WPI library.
	 * TODO:
	 * - Test to see if this works.
	 */
	void UpdateDashboard(void)
	{	
		// Setup here (default values set):
		const char *watchdogCheck;
		if (GetWatchdog().IsAlive()) {
			watchdogCheck = GetWatchdog().GetEnabled() ? "Enabled" : "DISABLED";
		} else {
			watchdogCheck = "DEAD";
		}
		
		const char *systemState;
		if (IsOperatorControl()) {
			systemState = "Teleoperate";
		} else if (IsAutonomous()) {
			systemState = "Autonomous";
		} else {
			systemState = "???";
		}
		
		const char *minibotStatus;
		float currentTime = timer.Get();
		if (currentTime >= (GAMEPLAY_TIME - 15)) { 
			minibotStatus = "Get Ready";
			if (currentTime >= (GAMEPLAY_TIME - 10)) {
				minibotStatus = "DEPLOY";
			}
		} else {
			minibotStatus = "...";
		}
		
		
		// Safety info
		SmartDashboard::Log(isSafetyModeOn ? "ENABLED" : "Disabled", "Safety mode: ");
		SmartDashboard::Log(watchdogCheck, "Watchdog State: ");
		
		// Robot actions
		SmartDashboard::Log(isFastSpeedOn ? "Fast" : "Normal", "Speed: ");
		SmartDashboard::Log(lift->getCurrentHeight(), "Current Lift Height: ");
		
		// Info about the field state
		SmartDashboard::Log(systemState, "System State: ");
		SmartDashboard::Log(IsEnabled() ? "Enabled" : "DISABLED", "Robot State: ");
		
		// Info about time
		SmartDashboard::Log(minibotStatus, "MINIBOT ALERT: ");
	}

	
	
	
	/****************************************
	 * UpdateDashboard:
	 * Overloading: Updates the dashboard, but with text also.
	 * Input = string to be displayed.
	 * Output = See UpdateDashboard(void)
	 * 			String from program.
	 * TODO:
	 * - Test
	 */
	void UpdateDashboard(const char *outputText)
	{
		// Call to base dashboard updater.
		UpdateDashboard();

		SmartDashboard::Log(outputText, "Message:");
	}
	
	
	
	
};

START_ROBOT_CLASS(MainRobot);


/*-------------------- Recommended Maximum Length of Lines -------------------*/
// END OF DOCUMENT
