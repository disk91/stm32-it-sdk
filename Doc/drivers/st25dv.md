# ST25DV - NFC Driver
ST25DV is a ST chip managing NFC with extended feature. It is using the NFC ISO-15693 standard. This standard is a bit exotic and one of the difficulty will be to find a compatible reader.
The device manages 4 zones of memory where you can set different access mode and password security settings.

## Setup the ST25DV
As all the drivers you need to enable it and setup it in the _configDrivers.h_
```C
// ------------------------------------------------------------------------
// NFC Tag : ST25DV

#define ITSDK_DRIVERS_ST25DV_I2C hi2c1 // I2C port to be used for communications
#define ITSDK_DRIVERS_ST25DV_GPO_BANK __BANK_B // GPO Pin
#define ITSDK_DRIVERS_ST25DV_GPO_PIN __LP_GPIO_5 // __LP_GPIO_NONE if not used
#define ITSDK_DRIVERS_ST25DV_LPD_BANK __BANK_B // LPD Pin
#define ITSDK_DRIVERS_ST25DV_LPD_PIN __LP_GPIO_6 // __LP_GPIO_NONE if not used
#define ITSDK_DRIVERS_ST25DV_I2C_PASSWORD 0x0000000000000001 // changeme => I2C password will be setup on device when != 0

// RF Password are 0000000000000000 by default. Only RF can set these password
// Only the first 1KB is accessible with a standard NFC read
// for the rest of the memory area an extended read is needed.
// For a larger reader compatibility, I assume it is better to have
// the Zone 1 & Zone 2 under this 1KB limit.

#define ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE 256 // Size in byte for User Zone 1 - This zone have no security option
#define ITSDK_DRIVERS_ST25DV_USER_Z1_ACCESS _ST25DV_ACCESS_RW_OPEN // Zone 1 is read only
#define ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE 768 // Size in byte for User Zone 2
#define ITSDK_DRIVERS_ST25DV_USER_Z2_PASS __ENABLE
#define ITSDK_DRIVERS_ST25DV_USER_Z2_ACCESS _ST25DV_ACCESS_RO_OPEN // Zone 2 is read only
#define ITSDK_DRIVERS_ST25DV_USER_Z3_SIZE 4096 // Size in byte for User Zone 2
#define ITSDK_DRIVERS_ST25DV_USER_Z3_PASS __ENABLE
#define ITSDK_DRIVERS_ST25DV_USER_Z3_ACCESS _ST25DV_ACCESS_RW_RWSECURED // Zone 3 is RW both secured by a password
#define ITSDK_DRIVERS_ST25DV_USER_Z4_SIZE 3072 // The reality is Zone 4 is up the memory size.
#define ITSDK_DRIVERS_ST25DV_USER_Z4_PASS __ENABLE
#define ITSDK_DRIVERS_ST25DV_USER_Z4_ACCESS _ST25DV_ACCESS_RW_RWSECURED // Zone 4 is RW both secured by a password
#define ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE ST25DV_USERZONE_1 // The serial communication module on User Zone is using Zone 1
#define ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET 128 // offset in byte in the USER Zone
```

* The LDP (Low Power) can be unactivated by setting the PIN __LP_GPIO_NONE
* The GPO (interruption) can be unactivated by setting the PIN __LP_GPIO_NONE
* The Zone definition is set with the different ..._USER_Zx_SIZE in byte. The driver will compute the different address automatically. 
* The I2C password can be changed by set it to the desired value. The default value is 0
* The RF password can only be set from the RF side. You will need to set them directly with a compatible reader.

> In you program loop you **must** add the call to _void  st25dv_process()_ to process the interrupts.

## SerialUz
The serial UZ feature is a serial port over NFC memory. This serial port is using a buffer to Read & Write data ; half duplex. 
```C
#define ITSDK_DRIVERS_ST25DV_WITH_SERIALUZ __ENABLE // Activate the serial User communication code
```
Is enabling the serial port. If the Console module is activated it is possible to connect the serial console to the NFC serial port.
This is done by selecting as console port **__UART_CUSTOM** This type of uart is a user defined uart based on weak function call.

The serial Uz port is quite slow and have a limited buffer for communication but works for most of the configuration operation and on field debugging.

## Bloc Access
The drivers allows to read & write bloc into the NFC memory zones.
```C
drivers_st25dv_ret_e drivers_st25dv_blocWrite(drivers_st25dv_zone_e zone, uint8_t blockId, uint8_t * data, uint8_t sz);
drivers_st25dv_ret_e drivers_st25dv_blocRead(drivers_st25dv_zone_e zone, uint8_t blockId, uint8_t * data, uint8_t sz);
```
You can easily create circular buffer on top of this interface.
From the RF side, the Block 0 to 0xFF can be accessed with standard NFC command. The next block needs to use the extended Nfc command. Make sure your reader SDK allows to use them.

## FTM
The Fast Transfer Mode is allowing a fast serial communication over NFC. It is actually not yet supported. It will be a different way to implement a serial console over FTM.

## Side notes
>The ST25DV is a bit a mess because it does not manage RF interface and I2C interface in parallel. It bases the access on First Come First Serve. For this reason the library is implementing retry on the I2C communication. 
When making your RF client application you also need to retry your communications but you also need to let time for the MCU to access the I2C bus.

>The ST25DV low power switch (using LDP pin) is flushing most of the configuration and requires a lot of operations a wake up. The interrupt generation in LDP mode is also limited. The driver detects the field and maintain a wake up step until being out of field with a security timer on top of the FieldOut interrupt. Some reader are enabling/disabling field between access. For this reason the FieldOut interrupt do not immediately switching the device LPD.

> The interrupt are proceeded synchronously in the  *st25dv_process()* the purpose is to avoid an asynchronous non mastered I2C call tsunami from the driver.