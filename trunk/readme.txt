===============================================================================
>> README <<
Skyline High School (with Boeing)
Spartabots, Team 2976
Program designed for 2011 FRC Robotics competition - Logomotion
Last updated Sunday, February 13th, 2011.

Table of Contents
  Instructions on how to control the robot
  Explaination of programming design choices
===============================================================================
Instructions on how to control the robot:

  Moving (joystick 1):
    Up, down, left, right	- Tilt joystick
    Clockwise				- Button 3 (top center)
    Counter clockwise		- Button 4 (top left)
    Moving fast				- Hold trigger (release for normal speed)
    Alter rotation speed	- Throttle (bottom middle, the twiddly thing)
 
    Turning safety on		- Button 6 (bottom left, further from driver)
    Turning safety off		- Button 7 (bottom left, closer to driver) 
    
    Deploy minibot			- Button 11 (bottom right, further from driver)
    Retract minibot			- Button 10 (bottom right, closer to driver)
    
  Lifting (joystick 2):
    Up, down				- Tilt joystick
    Preset 0 (lowest)		- Button 2 (top, closest to driver)
    Preset 1 (to 1st peg)	- Button 4 (top left)
    Preset 2 (to 2nd peg)	- Button 3 (top middle)
    Preset 3 (to 3rd peg)	- Button 5 (top right)
    
    Turning safety on		- Button 6 (bottom left, further from driver)
    Turning safety off		- Button 7 (bottom left, closer to driver) 
    
  Other infomation:
    Division of roles:
      There are two roles - moving the lift and moving the robot.  The first 
      joystick controls moving the robot.  The second joystick  controls moving 
      the lift.  They both can enable and disable safety mode.
    Safety:
      Turning safety on prevents moving at fast speeds - the trigger button
      will do nothing.  If the height of the lift goes too high, safety is 
      enabled, but is disabled once it lowers more.  If safety is enabled, it 
      will never disable until you press the correct button.  Turn safety on
      only for demos.
    Speed:
      There are two speeds - fast and normal.  You MUST HOLD THE TRIGGER to use
      full speed.  Normal speed is half of full speed.
    Minibot: 
      Deploying the minibot causes the rack to slide out.  For safety, it is 
      recommended to retract it afterwards.
    Autonomous:
      Refrain from touching the controls during autonomous mode.  If the robot
      will not enter operator control (probably due to a glitch in the field
      controls), either disable the safety or hold the trigger and jerk any
      joystick.  This will manually start operator controlled mode.
    Presets:
      The robot can automatically move the lift to any of the three pegs.  Just 
      press any of the top buttons on the 2nd joystick.  Selecting another 
      preset or moving the joystick interrupts the automatic movement, so don't 
      tilt the joystick when it's using a preset unless you want to.
    
===============================================================================
Explaination of programming design choices:
  Buttons:
    Toggles were deliberately avoided because they have a tendency to screw
    the user over.  To prevent ambiguity, each button does one thing and one
    thing only.
  Division of roles:
    The roles were divided so two drivers could comfortably control the robot.
  Rotation:
    The button for turning clockwise is the center button, not the right-side 
    button.  This is because the distance between the left and right buttons
    is too large for comfort and would have decreased reaction time.
  Scissor-lift:
    When examining the code (especially prior revisions), one may come across
    references to a 'scissor-lift' which seem to be shortened to 'lift' in later
    revisions.  Previously, the lifting mechanism was something called a scissor
    lift, but it proved to be a unsuccessful design and was replaced by a 
    simpler fork-lift mechanism.
  
===============================================================================
END OF DOCUMENT

