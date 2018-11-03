# Disk91 STM32 SDK

This project is a low level SDK for STM32 for making IoT devices.
It implements different usefull function is (I hope) a cleaner code than the usual ST SDK components. This SDK try to be fully configurable with header files.

Most is done to preserve code size. 
Made for being compiled with open-source environment GCC / AC6

* Supported functions
..* Low Power switch with regular auto-wakeup / LPUART / GPIO Wake-up
..* RTC
..* Logger
..* EEprom for configuration backup
..* Task Scheduler
..* State Machine 
..* Watchdog 

* Supported / tested platforms
..* STM32L011
..* STM32L053

The second objective is to be able to port this SDK to different patform
to make it a portable SDK. The SDK have a it-sdk directory where everything needs to be portable. stm32l-sdk contains all the subfunctions specific to this platform.

# Start your project by configuring a skeleton with Cube Mx 

* Sys Clock Mux : HSI16
* watchdog : configure IWDG with LSI 37KHz, IWDG_WINDOW 4095, IWDG_PRESCALER IWDG_PRESCALER_256
* rtc timer : TIM21 is enable with clock source internal
* lowPower : 
..* LPUART1 (Async 9600 - 8/N/1) WakeUp => clock mode need to be HSI, max speed 9600
..* RTC WakeUp => Activate ClkSource, Calendar, Internal WakeUp, Clk config : LSI, RTC/NVIC => Interrupt activated
..* GPIO WakeUp => Activate the GPIO as ExtInterrupt, set with Pull & Trigger en Fall/Rise, activate NVIC EXI1_15
* adc : Select an adc like for temperature to get the needed headers

When generating the Project
* Code generator:
..* select Generate peripheral initalization as par of .c/.h
..* Set all free pins as analog
* Advanced Settings:
..* Check Not Generated Function Call for ADC


# Import the SDK (this repository)

1. Clone this repository into the root of your project.
2. Add in project properties >> C/C++ General >> Path&Symbol >> Source location the repository ItSdk directory.
3. Add in project properties >> C/C++ Build >> Tool Settings >> MCU GCC Compiler >> Includes the ItSdk >> Inc directory.
4. Copy *ItSdk/Src/project_main.c.template* file into Core/Src/project_main.c and make the modification you want to get started. 

# Configure the SDK

1. Copy the *ItSdk/Inc/it_sdk/config.h.template* file into *Core/Inc/it_sdk/config.h*
2. Edit this file and fill the different settings according to your environment and your choices.


# Modify the Cube Mx skeleton
Things to not forget once a cubeMx project has been created
* Gpio.c
...=> remove the line with the default setting for GPIOs
  
* Main.c
...=> includes
```C
  /* USER CODE BEGIN Includes */
	#include <it_sdk/config.h>
	#include <it_sdk/itsdk.h>
	#include <it_sdk/logger/logger.h>
  /* USER CODE END Includes */
```  
...=> main
```C
    MX_IWDG_Init();
  	/* USER CODE BEGIN 2 */
  	  itsdk_setup();
  	/* USER CODE END 2 */

  	/* Infinite loop */
  	/* USER CODE BEGIN WHILE */
  	while (1)
  	{
	  /* USER CODE END WHILE */
	  /* USER CODE BEGIN 3 */
	  itsdk_loop();
  	}
  	/* USER CODE END 3 */
```

...=> error handler
```C
  void _Error_Handler(char *file, int line)
  {
	  /* USER CODE BEGIN Error_Handler_Debug */
	  #if ITSDK_LOGGER_CONF > 0
	     log_debug("Error : %s (%d)\r\n",file,line);
	  #endif
	  /* User can add his own implementation to report the HAL error return state */
	  while(1)
	  {
	  }
	  /* USER CODE END Error_Handler_Debug */
  }
```
  
Other modifications (need to be done on every CubeMx project regeneration):
  - Cube Mx is setting/resetting the Gpio state on init. You need to manually comment the line in *gpio.c* to avoid the pin to be modified on MCU wake-up

  - To support ADC: remove generaed adc.c/h and remove adc references in main.c
  
# License 

This code and ItSdk are under GPLv3. You can use it freely, you can modify, redistribute but *you must* to publish your source code. Other licenses can be obtained by contacting me on [disk91.com](https://www.disk91.com)
  