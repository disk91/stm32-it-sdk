# Accel - Accelerometer abstraction library
The accel library is an abstraction on top of different accelerometer drivers. The driver is selected in the _configDrivers.h_ file.
The library offer two main features
- Tilt / position / click detection
- Data capture
The driver deals with the underlaying accelerometer configuration. By-the-way a good understanding of the underlaying driver is important to have a correct behavior of the accelerometer as each of the sensors may have some particular behavior.

## Enable th accelerometer in configDrivers.h
Tune the settings according to your needs to preserve memory footprint
```C
// *************************************** ACCELEROMETERS *****************************************************************

// -------------------------------------------------------------------------
// Accelerometers : COMMON

#define ITSDK_DRIVERS_WITH_ACCEL_DRIVER				__ENABLE				// Enable ACCELEROMETER code
#define ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_SZ		64						// Buffer size to store the pending accelerometer data
																			//  x6 Bytes / Must be a power of 2
#define ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_WTM	32						// Software Watermark to process the DATABLOCK Transfert to
																			//  the application layer
#define ITSDK_DRIVERS_ACCEL_WITH_ANGLE				__DISABLE				// Movement angle determination code activated / deactivated (save space)


```

## Make the Accelerometer working
At first you need to call _itsdk_accel_ret_e accel_initPowerDown()_ from your **project_setup** function.
Then you need to call the _void accel_process_loop(void)_ in your **project_loop** function. This last function is already added in the _itsdk_loop()_ so you don't need to add it in the application layer. 


## Tilt / position / click detection

The following code, as an exemple initialize the accelerometer for these event detection:
```C
accel_configMovementDetection(
			200,		// Tilt detection Force level in mg
			50,			// Expected accuracy of the detection force level here we have from 150 to 250 mg
			200,		// Minimal duration of the event in ms
			30000,		// after >=30 seconds w/o movement a ACCEL_TRIGGER_ON_NOMOVEMENT is fired (latency related to RTC wakeup period)
						//   0 when not used.
			BOOL_TRUE,	// Enable the HP filter
			ACCEL_TRIGGER_ON_XYZ_HIGH | ACCEL_TRIGGER_ON_ANYPOS | ACCEL_TRIGGER_ON_DBLCLICK_XYZ | ACCEL_TRIGGER_ON_NOMOVEMENT // List of expected triger to respond to
		)
``` 

The list of the possible trigger is:
```C
typedef enum {
	ACCEL_TRIGGER_ON_NONE           = 0x0000000,		// Tilt detection
	ACCEL_TRIGGER_ON_X_HIGH         = 0x0000001,		//  Higher than the expected value
	ACCEL_TRIGGER_ON_X_LOW          = 0x0000002,		//  Lower than the expected value
	ACCEL_TRIGGER_ON_Y_HIGH         = 0x0000004,
	ACCEL_TRIGGER_ON_Y_LOW          = 0x0000008,
	ACCEL_TRIGGER_ON_Z_HIGH         = 0x0000010,
	ACCEL_TRIGGER_ON_Z_LOW          = 0x0000020,
	ACCEL_TRIGGER_ON_XYZ_HIGH       = 0x0000015,
	ACCEL_TRIGGER_ON_XYZ_LOW        = 0x000002A,
	ACCEL_TRIGGER_ON_X              = 0x0000003,
	ACCEL_TRIGGER_ON_Y              = 0x000000C,
	ACCEL_TRIGGER_ON_Z              = 0x0000030,
	ACCEL_TRIGGER_ON_XYZ            = 0x000003F,

	ACCEL_TRIGGER_ON_CLICK_X_P      = 0x0000100,		// P for positive force (normal side)
	ACCEL_TRIGGER_ON_DBLCLICK_X_P   = 0x0000200,
	ACCEL_TRIGGER_ON_CLICK_Y_P      = 0x0000400,
	ACCEL_TRIGGER_ON_DBLCLICK_Y_P   = 0x0000800,
	ACCEL_TRIGGER_ON_CLICK_Z_P      = 0x0001000,
	ACCEL_TRIGGER_ON_DBLCLICK_Z_P   = 0x0002000,
	ACCEL_TRIGGER_ON_CLICK_X_N      = 0x0004000,		// N for positive force (opposite side)
	ACCEL_TRIGGER_ON_DBLCLICK_X_N   = 0x0008000,
	ACCEL_TRIGGER_ON_CLICK_Y_N      = 0x0010000,
	ACCEL_TRIGGER_ON_DBLCLICK_Y_N   = 0x0020800,
	ACCEL_TRIGGER_ON_CLICK_Z_N      = 0x0040000,
	ACCEL_TRIGGER_ON_DBLCLICK_Z_N   = 0x0080000,
	ACCEL_TRIGGER_ON_CLICK_XYZ      = 0x0055500,
	ACCEL_TRIGGER_ON_DBLCLICK_XYZ   = 0x00AAA00,
	ACCEL_TRIGGER_ON_ANYCLICK       = 0x00FFF00,

	ACCEL_TRIGGER_ON_POS_TOP        = 0x0100000,		// Position detection, callback on position changed
	ACCEL_TRIGGER_ON_POS_BOTTOM     = 0x0200000,
	ACCEL_TRIGGER_ON_POS_LEFT       = 0x0400000,
	ACCEL_TRIGGER_ON_POS_RIGHT      = 0x0800000,
	ACCEL_TRIGGER_ON_POS_FRONT      = 0x1000000,
	ACCEL_TRIGGER_ON_POS_BACK       = 0x2000000,
	ACCEL_TRIGGER_ON_ANYPOS         = 0x3F00000,

	ACCEL_TRIGGER_ON_NOMOVEMENT     = 0x8000000			// No movement after the given time
} itsdk_accel_trigger_e;
```

When a event is detected by the underlaying driver, from an interrupt call, this interrupt will store the event list in an internal circular buffer. This is to avoid long interruption processing.
The _void accel_process(void)_ will process the circular buffer. 

The application needs to register eventHandler like this one:
```C
itsdk_accel_eventHandler_t accelEventCallback = {
		ACCEL_TRIGGER_ON_XYZ_HIGH | ACCEL_TRIGGER_ON_ANYPOS | ACCEL_TRIGGER_ON_DBLCLICK_XYZ | ACCEL_TRIGGER_ON_NOMOVEMENT,
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

	if ( triggers & ACCEL_TRIGGER_ON_NOMOVEMENT ) log_info("No Movement ");
	log_info("\r\n");
}
```

## Data Capture

The capture interface allow to asynchronously get a dataset in the application layer. A callback is performed to transfer the dataset from the acce driver to the application layer.
The capture process is initialized the following way:
```C
itsdk_accel_ret_e accel_startMovementCapture(
		itsdk_accel_scale_e         scale,			// Movement Scale 2G, 4G...
		itsdk_accel_frequency_e     frequency,		// Capture sampling rate
		itsdk_accel_precision_e     precision,		// Raw Data size 8B, 10B...
		itsdk_accel_trigger_e       axis,			// List of activated axis
		itsdk_accel_hpf_e           hpf, 			// enable High pass filter
		uint16_t                    dataBlock,		// Expected data to be returned by callback
		uint16_t                    captureBlock,	// Number of blocks to capture - 0 for non stop
		itsdk_accel_dataFormat_e    format,			// Data format
		itsdk_accel_data_t   *      targetBuffer,	// Data buffer to be used to push the data
		void (* callback)(itsdk_accel_data_t * data, itsdk_accel_dataFormat_e format, uint8_t count, itsdk_bool_e overrun)
													// callback function the interrupt will call
)
```
The capture interface will call _callback_ the number of times indicated in **captureBlock** (0 for infinitly). Each of the call will report **dataBlock** quantity of data points in the given **format**. The datapoint will be fill in the **targetBuffer**. This buffer must be a global table of **dataBlock** size. The overrun flag is position when in this dataset some of the data has been missed in any of the driver layer.

The **targetBuffer** is fill of datapoint in one of the expected format:
```C
typedef enum {
	ACCEL_DATAFORMAT_XYZ_RAW    = 0,		// data tab contains 16b raw data from the accelerometer
	ACCEL_DATAFORMAT_XYZ_MG     = 1,		// data tab contains 16b data converted in Mg
	ACCEL_DATAFORMAT_FORCE_MG   = 2,		// data tab contains in X the force vector
	ACCEL_DATAFORMAT_ANGLES_RAD = 3,		// data tab contains the angle of the device in milli-radian
	ACCEL_DATAFORMAT_ANGLES_DEG = 4,		// data tab contains the angle of the device in degrees
	ACCEL_DATAFORMAT_AVG_RAW    = 5,		// the block of data is averaged, first entry have the average RAW mode
											//  then you can call the converter from the application

	ACCEL_DATAFORMAT_END        = 0XFF
} itsdk_accel_dataFormat_e;
```

Each of the data point have the following structure but the content can vary depends on the expected **format**:
```C
typedef struct {         // RAW  MG  FORCE    ANGLE
    int16_t x;           //  X   X   Force    ROLL
    int16_t y;           //  Y   Y   0        PITCH
    int16_t z;           //  Z   Z   0
} itsdk_accel_data_t;
```

Cature stops when the **captureBlock** capture has been made or when the application call the following function:
```C
itsdk_accel_ret_e accel_stopMovementCapture(void);
```

The capture works the following way:
- A background process capture the data at the desired frequency. Data will be retrived from the accelerometer when the FIFO watermark level will be reached. If an interrupt line is connected the process will eventually wake up the device from deep-sleep. If not, the data will be retrieved at device regular wake up. You need to setup a correct wake-up frequency to not miss data. These data will be moved out from the hardware accelerometer to a circular buffer in the generic software accelerometer driver. This circular buffer size is defined in configDriver.h __ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_SZ__ it has to be a power of two number.
- Asynchronously, the data are then moved out from the circular buffer to the **targetBuffer** as soon as the **targetBuffer** is ready or, when this one is large, when the __ITSDK_DRIVERS_ACCEL_DATABLOCK_BUFFER_WTM__ level has been reached in the circular buffer. That way we can have a **dataBlock** request of any size. This asynchronous process is executed on device wake-up by executing the _void accel_process(void)_ function. That way the callback to the application layer is not executed inside the interruption. It means we can have long application level processing and in parallel to continue to capture next comming data from accelerometer under interruption.
- Before calling _callback_ the data are converted to the desired format.

This implementation creates a latency in data capture as we have a watermark level to reach in the driver to get the first data out. The software circular buffer and its watermark can also create an aperiodic data reporting for these periodic events. The driver makes its best to reduce this latency searching the best watermark level for the given parameter. Based on a search of the higest pgcd on the available watermark possible level. If the latency is an important point for your application, the best way is to increase the capture frequency to reduce this aspect or select a buffer size easily divisible. The objective is to have your expected capture period > accelerometer Watermark level period. 
If oversampling, you can average your data set with the associated **format** feature. If you need to convert the data, you can access the following converters from your **callback** function. 

```C
itsdk_accel_ret_e accel_convertDataPointRaw2Mg(itsdk_accel_data_t * data);
itsdk_accel_ret_e accel_convertDataPointMg2Force(itsdk_accel_data_t * data);
itsdk_accel_ret_e accel_convertDataPointRaw2Angle(itsdk_accel_data_t * data, itsdk_bool_e toDegrees);
```

## LIS2DH12 Drivers
The drivers can work with interruption line or without. If the interruption line are not connected the scrutation will be performed in the main loop. You have to ensure your device wakes-up at the desired frequency to catch the events and extract data from the Fifo before it overrun. The default watermark is from 4 to 28 values on the 32 available fifo spaces.



