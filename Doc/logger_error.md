# Logger & Error handler

The ItSdk has different modules to manage the errors and trace.

## Serial Logger



## Error report
The errors can be reported and store in e NVM memory for being consulted later for analysis. The type of error is composed by a list of 64b error blocks. Each block is composed by a 32b time entry in S followed by a 32b error code. 

The error code are composed the following way:
```
// +----+---+---+--------------------+----------------+
// | XX | Y | Z | AAAA AAAA AAAA AAAA| KKKK KKKK KKKK |
// +----+---+---+--------------------+----------------+
//
// XX -> Level : 00 - Info / 01 - WARN / 10 - ERROR / 11 - FATAL
// Y  -> Category : 0 - SDK / 1 - Application
// Z  -> Value : 0 - No associated value / 1 - Associated value
// AAAA .... AAAA -> 16b associated value
// KKKK .... KKKK -> 12b error code
```

The SDK code are stored in the *it_sdk/logger/error.h* file.

### Enable Error report
The error capture code is activated in the *config.h* file with the following defines:

```C
#define ITSDK_WITH_ERROR_RPT		__ENABLE								// Enable the Error reporting code. The allow to store error code in the EEPROM
#define ITSDK_ERROR_BLOCKS			64										//  Max number of error block / 1 block stores 1 error and needs 8 Byte for storage.
																			//  The first block is header
```
The size of the buffer in the NVM is given in number of 64b blocks. There is one more block for headers. The maximum value is 65533 blocks. The usual first limitation is the eeprom size.


The error code list can be extended at the application level with the following define:
```C
#define ITSDK_WITH_ERROR_EXTENTION	__ENABLE								//  Add an application extension for error code in configError.h file
```
In addition you need to copy the *configError.h.template* file from *Inc/it_sdk/* directory into the application Inc directory as *configError.h* file.

### Report and error

In your code, to report an error, you need to call the function **ITSDK_ERROR_REPORT(error,value)**
This function receives the error code as parameter and the potential value to store within the error code.

### Error code
The error code contains an error code, an associated value and a level (plus other element as described above). The Level is important: if info, warn, error have nothing special but will be used in the reader for highlighting error, the **fatal** level will create a loop to stop or reset the device after WDG expiration.

### NVM Storage
When the **ITSDK_ERROR_USE_EPROM** define is **__ENABLE** the error code are stored in the EEPROM in the MCU.
The EEPROM have first the secure store when actuvated, then the ERROR code blocks and at the end the configuration.

It is possible to choose a different storage for the error codes. In this case the **ITSDK_ERROR_USE_EPROM** can be **__DISABLE** and the following functions needs to be overide:

You can replace the default EEPROM NVM storage with your own NVM storage solution. For doing this you need to override 4 functions declared as _weak_ in the source code.

```C
itsdk_error_ret_e _itsdk_error_readHeader(itsdk_error_head_t * header);
itsdk_error_ret_e _itsdk_error_writeHeader(itsdk_error_head_t * header);
itsdk_error_ret_e _itsdk_error_write(uint16_t blockId, itsdk_error_entry_t * entry);
itsdk_error_ret_e _itsdk_error_read(uint16_t blockId,itsdk_error_entry_t * e);
```

### Acces the error report history
The simpler way to access the error report is to activate the console. Two shortcut are available in the private part of the console:
- e : print the log history in the following format:
```C
//I 000000000000000 : 0x10001001 ( S 0x001 / 0x0001 )
  |     |                  |       |   |       |
  |     + Time in S        |       |   |       + associated value
  |                        |       |   + Error code (just the error code)
  |                        |       + S for SDK error code / A for APP error code
  |                        + 32b Error code
  + Level (Info)
```
- E : clean the log history



