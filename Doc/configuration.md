# Configuration

The ItSdk is a configuration based SDK. Most of the configuration is statically made in the different configuration files.
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
#if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
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

## Console usage

The console allows to print and change the configuration. When the console is activated, some new functions are added to print and change the configuration.

To print the configuration of the application specific part you need to override the following procedure

```C
void itsdk_config_app_printConfig();
```

You can extend the console operation to add configuration like any console extension. See *console.md* file.

## Configuration change from console

The console allows to change the configuration dynamically. When a setting has been changed it is not changed directly in the *itsdk_config* structure but in a **itsdk_config_shadow** structure. This is ensuring we have a stable configuration before to commit it.

Once the configuration has been completed, the commit to the **itsdk_config** structure is made by calling the following function:

```C
typedef enum {
	CONFIG_COMMIT_ONLY = 0,			// copy shadow to config, not more
	CONFIG_COMMIT_SAVE,				// copy shadow to config, save the config
	CONFIG_COMMIT_SAVE_REBOOT		// copy, save, reboot device
} itsdk_config_commit_mode_e;

itsdk_config_commitConfiguration(itsdk_config_commit_mode_e mode);
```

This function, depending on the mode, will save the new config in the EEPROM, eventually reboot the device. From the console, commit is saving. Reboot is manual.

This function calls a function you can overide in your application as Pre processing on config update. 
```C
itsdk_config_ret_e itsdk_config_app_commitConfiguration();
```
This function will return SUCCESS to accept the new configuration in shadow structure or FAILED. If Failed the configuration will not be applied.

## Configuration change from application code

You can follow the same principle described for the config:
- making the configuration change in the shadow config
- commit the configuration

You can also modify the configuration directly and flush the configuration:
- making the configuration change in the config
- flush the configuration with __itsdk_config_ret_e itsdk_config_flushConfig()__

This second method is less safe as the __itsdk_config_app_commitConfiguration()__ will be bypassed and this will clear all the pending modifications in the shadow configuration.


