# Accel - Accelerometer abstraction library
The accel library is an abstraction on top of different accelerometer drivers. The driver is selected in the _configDrivers.h_ file.
The library offer two main features
- Tilt / position / click detection
- Data capture
The driver deals with the underlaying accelerometer configuration. By-the-way a good understanding of the underlaying driver is important to have a correct behavior of the accelerometer as each of the sensors may have some particular behavior.

## Make the Accelerometer working
At first you need to call _itsdk_accel_ret_e accel_initPowerDown()_ from your **project_setup** function.
Then you need to call the _void accel_process(void)_ in your **project_loop** function.


## Tilt / position / click detection

The following code, as an exemple initialize the accelerometer for these event detection:
```C
accel_configMovementDetection(
			200,		// Tilt detection Force level in mg
			50,			// Expected accuracy of the detection force level here we have from 150 to 250 mg
			200,		// Minimal duration of the event in ms
			BOOL_TRUE,	// Enable the HP filter
			ACCEL_TRIGGER_ON_XYZ_HIGH | ACCEL_TRIGGER_ON_ANYPOS | ACCEL_TRIGGER_ON_DBLCLICK_XYZ // List of expected triger to respond to
		)
``` 

The list of the possible trigger is:
```C
typedef enum {
	ACCEL_TRIGGER_ON_NONE			= 0x0000000,		// Tilt detection
	ACCEL_TRIGGER_ON_X_HIGH			= 0x0000001,		//  Higher than the expected value
	ACCEL_TRIGGER_ON_X_LOW			= 0x0000002,		//  Lower than the expected value
	ACCEL_TRIGGER_ON_Y_HIGH			= 0x0000004,
	ACCEL_TRIGGER_ON_Y_LOW			= 0x0000008,
	ACCEL_TRIGGER_ON_Z_HIGH			= 0x0000010,
	ACCEL_TRIGGER_ON_Z_LOW			= 0x0000020,
	ACCEL_TRIGGER_ON_XYZ_HIGH		= 0x0000015,
	ACCEL_TRIGGER_ON_XYZ_LOW 		= 0x000002A,
	ACCEL_TRIGGER_ON_X				= 0x0000003,
	ACCEL_TRIGGER_ON_Y				= 0x000000C,
	ACCEL_TRIGGER_ON_Z				= 0x0000030,
	ACCEL_TRIGGER_ON_XYZ			= 0x000003F,

	ACCEL_TRIGGER_ON_CLICK_X_P		= 0x0000100,		// P for positive force (normal side)
	ACCEL_TRIGGER_ON_DBLCLICK_X_P	= 0x0000200,
	ACCEL_TRIGGER_ON_CLICK_Y_P		= 0x0000400,
	ACCEL_TRIGGER_ON_DBLCLICK_Y_P	= 0x0000800,
	ACCEL_TRIGGER_ON_CLICK_Z_P		= 0x0001000,
	ACCEL_TRIGGER_ON_DBLCLICK_Z_P	= 0x0002000,
	ACCEL_TRIGGER_ON_CLICK_X_N		= 0x0004000,		// N for positive force (opposite side)
	ACCEL_TRIGGER_ON_DBLCLICK_X_N	= 0x0008000,
	ACCEL_TRIGGER_ON_CLICK_Y_N		= 0x0010000,
	ACCEL_TRIGGER_ON_DBLCLICK_Y_N	= 0x0020800,
	ACCEL_TRIGGER_ON_CLICK_Z_N		= 0x0040000,
	ACCEL_TRIGGER_ON_DBLCLICK_Z_N	= 0x0080000,
	ACCEL_TRIGGER_ON_CLICK_XYZ		= 0x0055500,
	ACCEL_TRIGGER_ON_DBLCLICK_XYZ	= 0x00AAA00,
	ACCEL_TRIGGER_ON_ANYCLICK		= 0x00FFF00,

	ACCEL_TRIGGER_ON_POS_TOP		= 0x0100000,		// Position detection, callback on position changed
	ACCEL_TRIGGER_ON_POS_BOTTOM		= 0x0200000,
	ACCEL_TRIGGER_ON_POS_LEFT		= 0x0400000,
	ACCEL_TRIGGER_ON_POS_RIGHT		= 0x0800000,
	ACCEL_TRIGGER_ON_POS_FRONT		= 0x1000000,
	ACCEL_TRIGGER_ON_POS_BACK		= 0x2000000,
	ACCEL_TRIGGER_ON_ANYPOS			= 0x3F00000,
} itsdk_accel_trigger_e;
```

When a event is detected by the underlaying driver, from an interrupt call, this interrupt will store the event list in an internal circular buffer. This is to avoid long interruption processing.
The _void accel_process(void)_ will process the circular buffer. 

The application needs to register eventHandler like this one:
```C
itsdk_accel_eventHandler_t accelEventCallback = {
		ACCEL_TRIGGER_ON_XYZ_HIGH | ACCEL_TRIGGER_ON_ANYPOS | ACCEL_TRIGGER_ON_DBLCLICK_XYZ,
		processAcceCallback,
		NULL
};
```
The eventHandler indicate what triggers it wants to subscribe and the function to call for one of these events.
We can have as many eventhandler as we want. We can have multiple eventHandler for the same event.

To register an eventHandler you have to call the following function:
```C
accel_addTriggerCallBack(&accelEventCallback);
```

The _void accel_process(void)_ is processing the list of pending triggers and the list of eventHandle and call the associated function.
Here is an example of function:
```C
void processAcceCallback(itsdk_accel_trigger_e triggers) {
	log_info("trigger status: 0x%08X - ",triggers);
	if ( triggers & ACCEL_TRIGGER_ON_X) log_info("X ");
	if ( triggers & ACCEL_TRIGGER_ON_Y) log_info("Y ");
	if ( triggers & ACCEL_TRIGGER_ON_Z) log_info("Z ");

	if ( triggers & ACCEL_TRIGGER_ON_CLICK_X_P ) log_info("ClkX+ ");
	if ( triggers & ACCEL_TRIGGER_ON_CLICK_X_N ) log_info("ClkX- ");
	if ( triggers & ACCEL_TRIGGER_ON_CLICK_Y_P ) log_info("ClkY+ ");
	if ( triggers & ACCEL_TRIGGER_ON_CLICK_Y_N ) log_info("ClkY- ");
	if ( triggers & ACCEL_TRIGGER_ON_CLICK_Z_P ) log_info("ClkZ+ ");
	if ( triggers & ACCEL_TRIGGER_ON_CLICK_Z_N ) log_info("ClkZ- ");

	if ( triggers & ACCEL_TRIGGER_ON_DBLCLICK_X_P ) log_info("DClkX+ ");
	if ( triggers & ACCEL_TRIGGER_ON_DBLCLICK_X_N ) log_info("DClkX- ");
	if ( triggers & ACCEL_TRIGGER_ON_DBLCLICK_Y_P ) log_info("DClkY+ ");
	if ( triggers & ACCEL_TRIGGER_ON_DBLCLICK_Y_N ) log_info("DClkY- ");
	if ( triggers & ACCEL_TRIGGER_ON_DBLCLICK_Z_P ) log_info("DClkZ+ ");
	if ( triggers & ACCEL_TRIGGER_ON_DBLCLICK_Z_N ) log_info("DClkZ- ");

	if ( triggers & ACCEL_TRIGGER_ON_POS_TOP ) log_info("Top ");
	if ( triggers & ACCEL_TRIGGER_ON_POS_BOTTOM ) log_info("Bottom ");
	if ( triggers & ACCEL_TRIGGER_ON_POS_LEFT ) log_info("Left ");
	if ( triggers & ACCEL_TRIGGER_ON_POS_RIGHT ) log_info("Right ");
	if ( triggers & ACCEL_TRIGGER_ON_POS_FRONT ) log_info("Front ");
	if ( triggers & ACCEL_TRIGGER_ON_POS_BACK ) log_info("Back ");
	log_info("\r\n");
}
```

## Data Capture




