# EEPROM - Secure Store
The secure store allows to store different keys & sensitive information into the EEPROM.

The secured store is a group of 16B entries encrypted with AES. Its is used to store critical elements like encryption keys.
The data are secured with AES-ECB : the protection is not optimal but access time is efficient and memory footprint is low.

The master key needs to be stored also in the eeprom creating another weak point. The master key is protected by a static key composed from
different elements : deviceId, static key, dynamic key and static customizable code.

__This protection is not perfect against physical attack on the device but it makes the attack more complex and reduce the capacity to create a generic attack.__
 
## Secured Store structure

The secured store is a 16B table:
|Block0 - Configuration & Master Key |
|------------------------------------|
|Block1 - AES-Encrypted Console Key  |
|Block2 - AES-Encrypted Key 1        |
|...                                 |
|BlockN - AES-Encrypted Key N        |

- The Block0 defines the configuration the other blocks, it is not encrypted

> MasterKey = f( SECURE_STORE_STATIC_KEY[0..15], itsdk_getUniqId, Block0 Key ) where f can be override to force attacker to do reverse ingeneering.

- The Block 1 contains the console access encrypted key.

- The Block 1.. N are encrypted thanks to the MasterKey.
- Each of the bloc is 16B lenght


The number of blocks and the use of the block is defined in the _securestore.h_ file ; the block ID will change according to the
project configuration to limit the EEPROM footprint. A modification of the configuration can impact the BLOCK 2.. organization.

**When changing the configuration (like adding a new communication module) the Key order can be remap and the SecureStore need to be reset from its factory default** 

When the Master Key is changed in the block 0, all the configured Blocks from 2 to N will be re-encrypted with the new key.

## Block0 specific structure

Block0 contains the Store configuration, status and dynamic key.
  | Byte1 | Byte2 / Byte3 |  Byte 4  | Byte 5 / ... / Byte 15 |
  |-------|-------------- |----------|------------------------|
  | Size  | Used slots    | Reserved |  Dynamic Key           |
  
- The first byte is indicating the number of block available in the structure. The max size is 16 block. (4 bits (high) are reserved)
- The byte 2/3 are a bit field indicating if the corresponding block has been setup of not. The highest bit is corresponding to the block15.
- The byte 4 is reserved for future use.
- The Last 12 bytes contains the dynamic key used by the function F to generated the MasterKey. This key shall be uniq per device. 


## Configuration
- The _config.h_ **ITSDK_WITH_SECURESTORE** setting activate the SecureStore when set as **__ENABLE**.
- You can specify a custom number of USER custom blocks on top of the SDK predefined blocks by setting ITSDK_SECSTORE_USRBLOCK to the expected number of extra blocks. The SDK accepts from 0 to 7 user extra blocks.   
- The initial dynamic key is set in the _config.h_ file initializing the **ITSDK_SECSTORE_DEFKEY** 12 byte "random" value. Then you will be able to change this value through a console command.
- The initial console password (this password unlock the serial console) is set with **ITSDK_SECSTORE_CONSOLEKEY** define. The password can be changed from the console cmd later. If securestore is disable this define defines the static password.

## Customizaton
The MasterKey is derivated from the dynamic key and any source of your choice. To override the standard way to create the MasterKey, your need to write a new function: __void itsdk_secstore_generateMasterKey(uint8_t * dynamicKey,uint8_t * masterKey)__ in your code. This new function will replace the default one at compilation time. This function is called with _dynamicKey_ 12B value set from the secureStore and return the _masterKey_ values. _masterKey_ is an allocated 16B array. 
If you want to keep the default Key computation, at least, you need to change the __ITSDK_PROTECT_KEY__ define with your own value.

## Use
The secure store is automatically initialized with the default values. In a user point of view, the only two way to access it are:
* __itsdk_secStoreReturn itsdk_secstore_readBlock(itsdk_secStoreBlocks_e blockType, uint8_t * buffer)__;
* __itsdk_secStoreReturn itsdk_secstore_writeBlock(itsdk_secStoreBlocks_e blockType, uint8_t * buffer)__;

