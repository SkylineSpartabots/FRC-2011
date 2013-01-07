
#ifndef TYPE_DEFS_H_
#define TYPE_DEFS_H_

typedef enum			// The ports on the digital sidecar (motors)
{
	kPWMPort_1 = 1,
	kPWMPort_2 = 2,
	kPWMPort_3 = 3,
	kPWMPort_4 = 4,
	kPWMPort_5 = 5,
	kPWMPort_6 = 6,
	kPWMPort_7 = 7,
	kPWMPort_8 = 8,
	kPWMPort_9 = 9,
	kPWMPort_10 = 10
} PWMPorts;

typedef enum			// The ports for the digital IO (limit switches)
{
	kDigitalIO_1 = 1,
	kDigitalIO_2 = 2,
	kDigitalIO_3 = 3,
	kDigitalIO_4 = 4,
	kDigitalIO_5 = 5,
	kDigitalIO_6 = 6,
	kDigitalIO_7 = 7,
	kDigitalIO_8 = 8,
	kDigitalIO_9 = 9,
	kDigitalIO_10 = 10,
	kDigitalIO_11 = 11,
	kDigitalIO_12 = 12,
	kDigitalIO_13 = 13,
	kDigitalIO_14 = 14
} DigitalIO;

typedef enum			// For the joysticks.
{
	kUSBPort_1 = 1,
	kUSBPort_2 = 2
} USBPorts;

typedef enum			// Buttons on the joystick.
{
	kJSButton_1 = 1,	// Trigger.
	kJSButton_2 = 2,	// On top of stick (bottom)
	kJSButton_3 = 3,	// On top of stick (middle)
	kJSButton_4 = 4,	// On top of stick (left)
	kJSButton_5 = 5,	// On top of stick (right)
	kJSButton_6 = 6,	// Bottom left (further away)
	kJSButton_7 = 7,	// Bottom right (closer)
	kJSButton_8 = 8,	// Bottom front left button
	kJSButton_9 = 9,	// Bottom front right button
	kJSButton_10 = 10,	// Bottom right (closer)
	kJSButton_11 = 11,	// Bottom right (further away)
	kJSButton_12 = 12,	// Doesn't exist
	kJSButton_13 = 13,	// Doesn't exist
	kJSButton_14 = 14	// Doesn't exist
} JoyStickButtons;

typedef enum			// The ports on the digital sidecar (motors)
{
	Error = -1,
	Nominal = 0,
	Success = 1
} TriState;
	
#endif	// TYPE_DEFS_H_
