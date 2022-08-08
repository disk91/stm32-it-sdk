# SI72XX - Hall Effect Magnetic Position and Temperature Sensor Driver

The Si7210 family of Hall effect magnetic sensors from Silicon Labs combines a 
chopper-stabilized Hall element with a low-noise analog amplifier, 13-bit
analog-to-digital converter, and an I2C interface. Leveraging Silicon Labs' proven 
CMOS design techniques, the Si7210 family incorporates digital signal processing 
to provide precise compensation for temperature and offset drift.

## Sources
 - Datasheet: https://www.silabs.com/documents/public/data-sheets/si7210-datasheet.pdf
 - Application Note AN1018: https://www.silabs.com/documents/public/application-notes/an1018-si72xx-sensors.pdf 

## Setup the SI72XX
As all the drivers you need to enable it and setup it in the _configDrivers.h_

## Using the SI72XX

### SI72XX HW versions


Constant declared in `drivers/hall/si72xx/si72xx.h`

```C
#define SI72XX_ADDR_0      0x30
#define SI72XX_ADDR_1      0x31
#define SI72XX_ADDR_2      0x32
#define SI72XX_ADDR_3      0x33
```

Based on the sensor version must be used proper I2C address:
 - SI72XX_ADDR_0 (0x30):
   - Si7210-B-00-IV(R)
   - Si7210-B-01-IV(R)
   - Si7210-B-10-IM2(R)
   - Si7210-B-11-IM2(R)
 - SI72XX_ADDR_1 (0x31): 
   - Si7210-B-02-IV(R)
   - Si7210-B-12-IM2(R)
 - SI72XX_ADDR_2 (0x32):
   - Si7210-B-03-IV(R)
   - Si7210-B-13-IM2(R)
 - SI72XX_ADDR_3 (0x33):
   - Si7210-B-04-IV(R)
   - Si7210-B-05-IV(R)
   - Si7210-B-14-IM2(R)
   - Si7210-B-15-IM2(R)
   

### Measurement of magnetic field

Read magnetic field with sensor SI72XX_ADDR_0 and go to sleep.

```C
...

int16_t data = 0;

if (Si72xx_ReadMagFieldDataAndSleep(SI72XX_ADDR_0, SI7210_20MT, SI72XX_SLEEP_MODE, &data) == __SI72XX_OK)
{
	return __SI72XX_ERROR;
}

return data;

...
```

### Measurement of temperature

Read temperature with sensor SI72XX_ADDR_0 and go to sleep.

```C
...

int32_t temp_mC = 0;		/* output in milli Celsius */

if (Si72xx_ReadTemperatureAndSleep(SI72XX_ADDR_0, &temp_mC) != __SI72XX_OK)
{
	return __SI72XX_ERROR;
}

return temp_mC;

...
```