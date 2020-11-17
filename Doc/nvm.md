# NVM - Non Volatile Memory

The SDK allows to use different type of NVM for different purpose. The mostly common used NVM is the MCU integrated EEPROM.

## MCU integrated EEPROM
The EEPROM is splet into 3 different zone, these zone will exists or not depending in the configuration.

```C
 Offset
  0x00    +----------------------------------+
          +           SECURE STORE           +
          +----------------------------------+
          +            ERROR LOG             +
          +----------------------------------+
          +          SIGFOX NVM AREA         +
          +----------------------------------+
          +          CONFIGURATION           +
          +----------------------------------+
          +            USER LAND             +
          +----------------------------------+
```

### Secure Store

The Secure Store allows to store critical informations like credentials. See *securestore.md* file for details.

### Error Log

The Error Log allows to store error code persistantly and access them later to debug you firmware.

### SIGFOX NVM AREA

Used by sigfox lib to store internal information like sequence number
Only activated when sigfox is enable

### Configuration

The configuration is composed into different parts:
* The configuration header - it maintains a Magic, a version and a CRC to ensure the validity of the data
* The sdk configuration structure - it contains the SDK configuration elements used by sdk
* The aplication configuration structure - it contains the application specif elements - basically it is empty

See *configuration.md* file for details

### end of the reserved zone 

The offset of the user free area can be obtained with the following function:
```C
bool eeprom_getPostConfigOffset(uint32_t * _offset);
```

### EEPROM zone change

The different boundaries can changed over compilation and versionning. All the zone are secured with MAGIC numbers. When the Magic are not found the zone will be restored from factory default. It means the previous settings will be lost.
This apply for version change also and version will be able to increase / decrease the size of a zone.

A SDK upgrade can also add new element in the configuration zone and re-initialize the entiere EEPROM. You need to make attention of this.

A a prevendive approach, the SDK config zone includes some reserved area to reduce this impact.


