# Debug with SEGGER RTT
Segger RTT allows to have a debug console over SWD debug port instead of a Serial port. It have a lot of limitation like when the debug si running deepsleep but it can help to debug and is really fast compared to Serail port as this system is just a memory print instead of a low speed serial write.

## Configure ITSDK for RTT debugging
To activate the SeggerRTT debugging you have different configuration steps to perform:
1. in your *config.h* file, activate it
```C
#define ITSDK_LOGGER_WITH_SEG_RTT	__ENABLE								// enable SEGGER RTT trace driver for DEBUG interface
#define ITSDK_LOGGER_CONF 0x000F											// The leading F is corresponding to Segger RTT interface
```
2. in _stm32-it-sdk/Src/drivers/SeggerRTT/_ you need to rename _SEGGER_RTT_ASM_ARMv7M2.S.txt_ into _SEGGER_RTT_ASM_ARMv7M2.S_ This is to avoid compilation problem and over configuration when you are not using RTT debuging.

3. Go in project preferences >> C/C++ General >> Paths and Symbols menu
- In the **Includes** tab you need to reacreate all the include structure existing for GNU C into Assembly
- In the **Symbol** tab you need to add the MCU target symbol you have in the GNU C into the Assembly one ex: STM32L072xx
- You need to do this for _debug_ and _release_

4. Compile your project.

## Use it
* You can connect to his console with RTTViewer tool, search the SEGGER bank indicating a starting address and a size. An exemple for STM32L072xx : 
```
0x2000000 20000 
```
* You also telnet on 19021 port