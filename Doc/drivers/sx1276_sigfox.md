# SX1276 / MURATA used as Sigfox

## Pre-requisites

### Hardware configuration : 
- TCXO powered is controled by a GPIO => This allows to keept it activ as we want
- PH0-OSC connected to TCXO-OUT => This allows to use TCXO as a HSE clock source. The purpose is to have a synchro between the MCU clock and the generated signal during signal transmission over DMA
- DIO4 connected to a GPIO => DIO4 is an interrupt channel for Sigfox mode it needs to be connected 
- SPI target frequency : 8Mb/s => SPI_BAUDRATEPRESCALER_4 (point to verify as the parameter pass to the ST function is 16M but retun corresponds to 8MBit/s)

- PA15 => SPI_NSS pin should be configured PULL_UP with Set as initial state


### CubeMx specific configuration :
__SPI1_TX DMA Channel__ needs to be enable in Cube MX. Go to SPI1, Add a DMA entry, Select SPI1_TX in the DMA windows.
This will generate the necessary IRQ Handler for SPI/DMA transmission.

