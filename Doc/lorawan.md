# LoRaWan driver
The SDK supports different LoRaWan implementation over a common interface. Actually the only supported implementation is SX1276 with Murata chip using the Semtech LoRaWan stack.

## Example
You can take a look at the Murata example : https://github.com/disk91/itsdk-example-murata-lora

## Enabling LoRaWan
To activate the LoRaWan feature available in the SDK, you need to enable it in the *config.h* file.
```C
#define ITSDK_WITH_LORAWAN_LIB __ENABLE // Include the lorawan code when 1 disabled when 0
#define ITSDK_LORAWAN_LIB __LORAWAN_SX1276
```
This is enabling the LoRaWan code and specific header inclusion and specify the hardware implementation.

The LoRaWan detailed configuration is defined in the *configLoRaWan.h* file. A template of this file need to bin imported from *Inc/it_sdk/* folder and copied in the *Inc* directory of the project. Read the header file to see how to setup the LoRaWan stack.

The LoRaWan stack supports the SDK end-to-end payload encryption. You have some extra settings concerning this part when activated.

## Setting-up LoRaWan
The function to setup LoRaWan stack receives a Region to initialises the stack and an associated channel configuration when you want to configure a specific one.
The other parameters like the DEVEUI, APPEUI, APPKEY are loaded directly by calling dedicated functions allowing to securely store these element from a NVM or to implement your own solution. Actually only  OTAA join method is implemented. ABP mode will come soon.
The Setup is preparing the LoRaWan lib or external devices by registering the keys into the library / modem.

Different function can be override to extract the keys from your code/NVM to initialize the LoRaWan stack like:
```C
itsdk_lorawan_return_t itsdk_lorawan_getDeviceEUI(uint8_t * devEui);
itsdk_lorawan_return_t  itsdk_lorawan_getAppEUI(uint8_t * appEui);
itsdk_lorawan_return_t  itsdk_lorawan_getAppKEY(uint8_t * appKey);
```
These function return the requested key from the storage you want. You can redefine these function in your own code.

By default, the functions are using the *secureStore* as a source for these informations when it is activated. Otherwise it uses the static defines existing in configLoRaWan.h. The secureStore is initialized with the static defines in configLoRaWan.h
When the *console* module is activated the *secureStore* settings can be changed from the console. 
This is allowing the following features:
- Use a specific default LoRaWan setting to make a communication test out of production.
- Change the configuration at production level or later with the device specific configuration. 
- Later change the configuration on the field from a serial console or any communication channel.  

The other parameters are set from the *configLoRaWan.h* file. Based on the configuration mode:
```C
ITSDK_CONFIGURATION_MODE == __CONFIG_STATIC		//  __CONFIG_MEMORY or __CONFIG_EEPROM
```
The setting will come from the static definition or the dynamic configuration given by the *sdk_config* structure. In this second situation the LoRaWan setting can be modified dynamicly thanks to the *console*. The keys and the other configuration mode are separated: you can have a dynamic key with the *secureStore* and a static configuration for mode like OTAA, ADR ... It's up to you.  

## Asynchronous / Synchronous approach
The join and transmission can be done as synchronous process or asynchronous process.
- with synchronous approach, the function will return once the join or communication is complete. Succeeding or failing. 
- with asynchronous approach, the function will return immediately. The system will be able to switch to low-power during the wait operations. As a consequence, the wake-up pins need to be correctly configured to wake-up the MCU in LoRaWan interruption. In asynchronous mode, callback function will be used to inform the application layer on the processing evolution.

## Joining procedure 
- In async mode, the join procedure is taking a callback function as parameter. All the parameters are coming from the configuration loaded in the previous setup step.
```C
itsdk_lorawan_join_t itsdk_lorawan_join_async(
     void (*callback_func)(itsdk_lorawan_join_t status)
   );  
```
The callback function will be called on every state change with the new state as parameter.
> LORAWAN_JOIN_FAILED
> LORAWAN_JOIN_SUCCESS

- In sync mode, the join procedure is returning the final status once the join process has completed.
```C
itsdk_lorawan_join_t  itsdk_lorawan_join_sync();
```
The status are the same as previously listed.

## Send & Receive
The reception is only possible as a response of a transmission. The API offer different **send** functions. Depending on mode, you can receive data as a response. 

### Asynchronous mode
The default function is a bit complex to support different mode:
```C
itsdk_lorawan_send_t itsdk_lorawan_send_async(
     uint8_t * payload,
     uint8_t payloadSize,
     uint8_t port,
     uint8_t dataRate,
     itsdk_lorawan_sendconf_t confirm,
     uint8_t retry,
     void (*callback_func)(itsdk_lorawan_send_t status, uint8_t port,     uint8_t size, uint8_t * rxData),
     itdsk_payload_encrypt_t encrypt // End to End encryption mode
); 
```
- The payload, payloadSize, port, dataRate defines the transmissions to be executed.
- The confirmation type allows to have an acknowledgement and downlink data.
- The reply only applies for acknowledged transmission: the message will be repeated until retry reached or confirmation received.
- The last parameters allows to enable the Sdk End-to-End encryption. This encryption layer is on top of the LoRaWan payload. It protects your data against the LoRaWan operator. AES and SPECK can be activated. Secrets keys are managed with the *secureStore* module and initialized with the static defines.

As for the Join procedure a callback function is used to report the communication progress. This callback also .Different states are reported:
> LORAWAN_SEND_SENT
> LORAWAN_SEND_ACKED
> LORAWAN_SEND_ACKED_WITH_DOWNLINK
> LORAWAN_SEND_ACKED_WITH_DOWNLINK_PENDING

When the *DOWNLINK_PENDING* is returned is means you have received data from the NOC and you have more data to retrieve. A new communication is required for this purpose.

You can request the current state of the communication with the following function:
```C
itsdk_lorawan_send_t  itsdk_lorawan_getSendState();
``` 
### Synchronous mode
In synchronous mode the function have the same kind of parameters and includes the downlink data buffers:
```C
itsdk_lorawan_send_t itsdk_lorawan_send_sync(
     uint8_t * payload,
     uint8_t payloadSize,
     uint8_t port,
     uint8_t dataRate,
     itsdk_lorawan_sendconf_t confirm,
     uint8_t retry,
     uint8_t * rPort, // In case of reception - Port (uint8_t)
     uint8_t * rSize, // In case of reception - Size (uint8_t) - init with buffer max size
     uint8_t * rData, // In case of recpetion - Data (uint8_t[] bcopied)
     itdsk_payload_encrypt_t encrypt // End to End encryption mode
)
```
The response status are the same as the one previously detailed.

To simplify the API, when you have a simple uplink transmission, you can use this second API:
```C
itsdk_lorawan_send_t itsdk_lorawan_send_simple_uplink_sync(
    uint8_t * payload,
    uint8_t payloadSize,
    uint8_t port,
    uint8_t dataRate
);
```