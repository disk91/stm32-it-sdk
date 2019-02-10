# Console - serial terminal for configuration & more

The console allow to dynamically get the device status, configuration & more. It also allow to configure the device from a Serial port. The console can be used for Production configuration as for on site debugging.

The console needs to be activated and configured to be operational. Once activated, the console is protected by a passwd. This password is set as part of the securestore. If securestore is not configured the password is static.

## Configuration
The define **ITSDK_WITH_CONSOLE** is activating the console code when its value is set to **__ENABLE**
The console is set on one of the serial port. The serial port is configured with the **ITSDK_CONSOLE_SERIAL** define. 

The selected UART need to be support Wake-Up from low power to make it working. As an example, with STM32, only a console on LPUART1 will work in LowPower mode. If the selected Serial is not compatible with low power you will need to have an external trigger to switch lowPower off.
The following example switch it of from an external button:

```C
void project_loop() {
	// ...
	if( gpio_read(BUTTON_PORT, BUTTON_PIN) == 0 ) {
		lowPower_disable();
		log_info("LowPowerOff\r\n");
		itsdk_delayMs(200);
	}
	//...
}
```

## Password
The password can be any char. The maximum size is 15 chars and it is ending by a 0.

