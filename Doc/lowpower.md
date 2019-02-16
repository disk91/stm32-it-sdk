# Low Power Mode


### UART Wake Up

#### STM32
On STM32 UART wake up is possible for any of the UART. The most important thing is to have UART CLK source (UART Source MUX) in CubeMX set to HSI or LSE. Depends on the MCU the possible UART/LPUART to wake up the device are changing.
- when HSI, the baudrate can be set to any rate
- when LSE, the baudrate should be under 9600 bps
The associated EXTI - Wakeup interrupt must be activated in CubeMx

To activate Uart wake up you need to have different settings in _config.h_ file:
 
- Activate the WakeUp on UART:
```C
#define ITSDK_LOWPOWER_MOD			( __LOWPWR_MODE_STOP       \
									| __LOWPWR_MODE_WAKE_RTC   \
									| __LOWPWR_MODE_WAKE_GPIO  \
									**| __LOWPWR_MODE_WAKE_LPUART** \
									)								
```							

- Add the UART Pin in the KEEP list (according to you pin setting):
```C
#define ITSDK_LOWPOWER_GPIO_A_KEEP	(  __LP_GPIO_1  \
									 | __LP_GPIO_2  \
								 	 | __LP_GPIO_3  ...

```

- Add the UART PIn in the WAKE List
```C
#define ITSDK_LOWPOWER_GPIO_A_WAKE	(  __LP_GPIO_1  \
									 | __LP_GPIO_2  \
								 	 | __LP_GPIO_3  ...

```


