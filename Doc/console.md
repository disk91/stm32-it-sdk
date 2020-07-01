# Console - serial terminal for configuration & more

The console allow to dynamically get the device status, configuration & more. It also allow to configure the device from a Serial port. The console can be used for Production configuration as for on site debugging.

The console needs to be activated and configured to be operational. Once activated, the console is protected by a passwd. This password is set as part of the securestore. If securestore is not configured the password is static.

## Hardware configuration
The console is usually a floatting connector usually disconnected. It sounds recommended to configure the RX line with a pull-up in a such situation to reduce the risk of noise over the serial line and unexpected interrupts including unexpected command execution.
The console usually wakes up the sleeping MCU, a specific configuration is required. 9600 bps is a maximum, the source clock needs to be HSI16... see low power documentation for details.

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


## Console behavior
- Until the session is unlocked, the string __password:__ is print in the console after a carriage return or any invalid password.
- Once the console is unlocked, the string __OK__ will be printed.
- The console will print __OK__ when a command is correctly executed
- The console will print __KO__ when a command is invalid or failed
- The session will end after **ITSDK_CONSOLE_EXPIRE_S** seconds from the last received command and automaticalli lock. 

## Console commands
There are two groups of console commands:
* The public command: they are accessible even if the console is locked
* The private command: tehy are accessible only when the console is unlocked
When a string is received in the console, the system will call a chain of operation processing. Any component of the SDK can register new actions to one of the two chains ( private and public ) this way the console code can be extended without modifying it. For this you define a static structure like the following one:

```C
typedef struct itsdk_console_chain_s {
	itsdk_console_return_e (*console_private)(char * buffer, uint8_t sz);	// function to proceed operations when console is unlocked
	itsdk_console_return_e (*console_public)(char * buffer, uint8_t sz);	// function to proceed operation whatever
	struct itsdk_console_chain_s * next;									// next in chain
} itsdk_console_chain_t;
```

- If one type of operation is not needed the corresponding function point will be NULL.
- The function must not modify the buffer content: it will be pushed to the next function in the chain
- Multiple function can respond to the same command. Like **?** command corresponding to help printing. Every function should add its help documentation as part of the chain. 
- Make sure the same command will not have two distincts operations.
- The size of the string is transmitted over _sz_ parameter. The string is ending with \0
- The function returns the execution status of the function:

```C
typedef enum {
	ITSDK_CONSOLE_SUCCES = 0,								// console command proceed with success
	ITSDK_CONSOLE_NOTFOUND,									// console command corresponding to no operation
	ITSDK_CONSOLE_FAILED									// console command corresponding to an operation but failed
} itsdk_console_return_e;
```

The console command are registered in the command chain with the **void itsdk_console_registerCommand(itsdk_console_chain_t * chain)** function. The corresponding chain will be added at end of the chain.

## Console default commands
The following list of commands are the one accessible once logged
```
--- Common
?          : print help
!          : print copyright
v          : print version
X          : exit console
R          : reset device
l / L      : switch LowPower ON / OFF
c          : print device config
s          : print device state
t          : print current time in S
T          : print current temperature in oC
b          : print battery level
B          : print VCC level
r          : print last Reset Cause
```

This list is reduced when disconnected
```
--- Common
?          : print help
!          : print copyright
v          : print version
```



