# Configuration

The ItSdk is a configuration based SDK. Most of the configuration is satically made in the different configuration files.
This document is about the dynamic configuration part: the way this static configuration is stored into the NVM memory and able to be updated dynamically at run.

## Configuration storage

If you don't need a dynamic configuration you can rely on static configuration and avoid to use the EEPROM of the MCU. For activating this option, you can *__ENABLE* it in the **config.h** file.

```C
#define ITSDK_WITH_NO_CONFIG_AT_ALL		__DISABLE
```

The option is *__DISABLE* by default, it means the SDK stores the configuration in the NVM

When you application have a specific configuration you can append this configuration to the SDK configuration and get all the benefits of the configuration management. To *__ENABLE* it you need to update the *config.h* file:

```C
#define ITSDK_WITH_CONFIGURATION_NVM	__ENABLE
```

This will require to create a new file *configNvm.h* file (even if you are not usig NVM storage and always restore the config from factory static values). This file have an existing template in the *Inc* directory.

## Configuration structure

The configuration is loaded at MCU setup and accessible from the RAM in the **itsdk_config** structure. This structure is described in *Inc/it_sdk/eeprom/sdk_config.h* it have two parts:

```C
typedef struct {
	itsdk_configuration_internal_t	sdk;
#if ITSDK_WITH_CONFIGURATION_NVM == __ENABLE
	itsdk_configuration_app_t		app;
#endif
} itsdk_configuration_nvm_t;
```

The sdk part must not be modified, it is part of the SDK configuration.
The app part is free to be modified. You need to preserve the minimal structure (version) required for config management. For EEPROM storage the size must be aligned on 32b address and an alignement structure need to be added to fullfill this contraint.

```C
typedef struct {
	uint8_t				version;		// configuration version

	uint8_t				alignment[3];	// For 32bits size alignment
} itsdk_configuration_app_t;
```

This structure will be initialized from the factory default by overriding the following function:

```C
itsdk_config_ret_e itsdk_config_app_resetToFactory();
```

