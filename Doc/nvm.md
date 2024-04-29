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


## EEPROM with STM32-WL

STM32WL does not have internal EEPROM and the NVM storage rely on FLASH, it has some impact, part of it the number of cycles limited to 10K per memory zone.

### STM32WL - Flash access constraints

STM32WLE5 have 256KB flash in page of 2KB each (128 pages from 0 to 127). It is possible to write words into pages a single time. Write mechanism can switch a 1bit to 0bit only. The STM32WL 
implementation does not allows to change the value for a different one even if only 1 to 0 bit are switched. That said, it's possible to switch all bits to 0 whatever was the previous value. 
The page reset process is switching a whole page to FF values. Each memory zone supports about 10K write, when a variable is stored in NVM like a frame counter, it's important to
not rewrite the same flash area to update it or the chip life will be reduced shortly. When a large structure is rewrited it's important to limit the write to the minimum zone.
Write access is per 64bits words.

For these reasons, this SDK uses an EEPROM emulator inspired by the ST EEPROM Emulation, even if different for making it simpler, less generic but potentially more efficient. 

The Soft EEPROM work with 2K pages containing vitual memory zone defined like this:

```C
   +----. 10 .---+---. 2 .---+---. 4 .---+-----. 112 .------+
   +    Address  +  Updates  +   Check   +   14 Bytes Data  +
   +-------------+-----------+-----------+------------------+
   
  
   Address
   10 bits allows 1024 entries for 14*1024 eeprom storage
   ability to create EEPROM up to 14KB
   
   address 0 can't be used, means Cleared line
   adresss 3FF can't be used, means empty line
   
   Updates
   Incremented up to 3 when a zone is overriden (used to identify zone never updated)
   
   Check
   Address + Data calculation to verify data coherence. Count the number of '1' in this payload % 64
   
      
   Total 128 Bits / 16 Bytes per line
   This allows 128 lines per 2K blocks - 1 for the header (eq 127)
   We keep 1 line (128b) for header
   
```

Each of the pages header is structured the following way:
```C
  +---. 16 .---+---. 14 .---+---. 2 .---+ +---. 8 .---+---. 24 .---+
  +    Magic   +    Aging   +  Reserved + +  Version  + Reservered +
  +------------+------------+-----------+ +-----------+------------+
  
  +----------------------------. 64 .------------------------------+
  +                          Page State H                          +
  +----------------------------------------------------------------+

  +----------------------------. 64 .------------------------------+
  +                          Page State L                          +
  +----------------------------------------------------------------+

  +----------------------------. 64 .------------------------------+
  +                          Reserved (alignment)                  +
  +----------------------------------------------------------------+
  
  Magic is 0x916E indicating the EEPROM page has been initialize
  Aging is incremented everytime the page is cleared and reuse, 10K is a maximum
  Version is soft eeprom version for later potential migration
  
  Page State
  Different states in life cycle:
  
 
              +---------------------------------------+
              \/                                      |
  FREE -> INIT_EMPTY -> MOVE_IN -> READY -> MOVE_OUT -+
    |          |                    ^
    +----------+--------------------+
  
  State H  State L
  0xFFFF / 0xFFFF - FREE			- The page is not initialized, blank memory
  0xFFFF / 0x9999 - INIT EMPTY		- The page as been init but not yet in use, maintain the aging of the page
  0xFFFF / 0x0000 - MOVE IN			- During garbage collection, indicate memory transfer to this page
  0xA5A5 / 0x0000 - READY			- The page is open for adding lines into it
  0x0000 / 0x0000 - MOVE OUT		- During Garbage collection, indicate source of transfer to a MOVE_IN page

```

### Write process

When a write process is starting, the address is recalculated to identify in what line it belongs, (addr = (addr / 14) + 1), then we need to
search for this address in the ready to be used pages. First found is the good one.

When the entry exists, we need to read this entry, ensure it differ, then create a new line where line is empty (value 0xFFFF...FFFF), then mark the 
previous line as cleared value 0x0000..0000 for first 128bits

When the entry does not exists, we create a new line where line is empty.

The unaffected bytes are automatically set to 0.

Due to the read & write process with 14 bytes long entries it's important to use the EEPROM with full structure update and not byte by byte write.

There are no orders in memory allocation. 

We need to make sure one of the page is kept empty for Garbage collection mechanism


### Read process

Compute the address and search for it, mask the requested data and return it. Nothing special.

### Garbage Collection

The EEPROM use more pages than what we make available in storage. Ratio is starting at 20% but it can be increased to reduce the number of 
page erasing cycles. It means for 6K user accessible memory 

| Accessible Memory | Total Need Memory | 2K Pages used | 2K Pages min total    | Min Real Memory |
| ----------------- | ----------------- | ------------- | --------------------- |-----------------|
| 4 KB              | 4.6KB             | 3             | 4                     | 8K              |
| 6 KB              | 6.9KB             | 4             | 5                     | 10K             | 
| 8 KB              | 9.2KB             | 5             | 6                     | 12K             |
| 16 KB             | 18.2KB            | 10            | 11                    | 22K             |

It's better to increase the total page when you are using most of the used pages or to over reserve eeprom size for a smaller storage need. Say it 
differently more space you reserve for a smaller storage, less write cycle you will perform. When EEPROM is used to store like transmission sequence 
number, it's really better to increase the storage space. Be careful with on reboot counter update that can burn Flash really shortly.

When no more space is available (1 page is still available) to store new line we need to clear and compact pages, algorithm seach best page to compact : 
- the one with the largest number of cleared line as source first
- as much as possible we try to reduce the number of used pages by selecting a group of pages fitting in a new pages

Destination is the empty page, when the empty page aging is > 100 and 2x the min aging, we switch the pages to free low aging pages for next writes and clean also the pages not changing a lot

### Garbage collection

When only one single page is free, we must start a garbage collection. This will compact the entries by removing the cleared lines.
We need to reduce the write cycle and avoid to have a major difference in term of aging. On top of this, certain value never change and
we may group them in pages to reduce the page modifications. (these pages will be later swapped to offer more write cycles for a given var.

Process:
- concat in one page the most cleared pages to free one or more.
- free the pages
- look for pages to swap when we have aging < avg_aging && max_aging > 2*avg_aging && max_aging > 100







 