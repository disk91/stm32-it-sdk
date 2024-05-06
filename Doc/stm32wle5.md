## STM32WL_E5 Specificities

### Low Power & Wake Up
- UART 1 & 2 allow char reception during sleep mode 0 & 1
- LPUART1 Generate a wakeup interrupt and works on sleep mode 0,1 & 2

### Sigfox integration
- git submodules needs to be loaded
- in the project properties,in C/C++ Build >> Setting >> Tool Settings >> MCU GCC Compiler >> Includes Path , add:
	```
	"${workspace_loc:/${ProjName}/stm32-it-sdk/3rdParties/sigfox/sigfox-ep-lib/inc}"
	"${workspace_loc:/${ProjName}/stm32-it-sdk/3rdParties/sigfox/sigfox-ep-rf-api-semtech-sx126x/inc}"
	"${workspace_loc:/${ProjName}/stm32-it-sdk/3rdParties/semtech/sx126x_driver/src}"
	```
- in the project properties, we need to enable the Sigfox flag file to be used in the C/C++ Build Parameters
	```
	USE_SIGFOX_EP_FLAGS_H
	```	
- in the project properties >> C/C++ General >> Path & Symbol >> Source location add `stm32-it-sdk/3rdParties` folder
	
#### Sigfox Tunning
The sigfox library can be setup for a lower memory & flash footprint, edit file in 3rdParties/sigfox-ep-lib/sigfox_ep_flags.h