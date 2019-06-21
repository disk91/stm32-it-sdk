# SX1276 / MURATA used as Sigfox

## Pre-requisites

### Hardware configuration (routing): 
* TCXO powered is controled by a GPIO => This allows to keept it activ as we want
* PH0-OSC connected to TCXO-OUT => This allows to use TCXO as a HSE clock source. The purpose is to have a synchro between the MCU clock and the generated signal during signal transmission over DMA
* DIO4 connected to a GPIO => DIO4 is an interrupt channel for Sigfox mode it needs to be connected 

### Cube Mx configuration
* The SPI pins need to be set including PA15 GPIO_SPEED_FREQ_VERY_HIGH => 32MHz
* PA15 (SPI_NSS)
  * SPI_NSS pin should be configured PULL_UP with Set as initial state
  * Speed is VERY_HIGH
* TIM2 is enable with Inetrnal Clock source / nothing specific it will be setup by the driver code
* SPI1 is enable with
  * Soft NSS
  * SPI_BAUDRATEPRESCALER_32 (it will be reconfigured by the driver code) 
  * Data Size 8b (it will be reconfigured by the driver code)
  * DMA Settings : Enable SPI_TX DMA Request on default channel (it will be reconfigured by the driver code)
  * Enable DMA1 channel2 & 3 interrupt
  * Disable SPI1 global interrupt
  * Set GPIO as Very High Speed
  


