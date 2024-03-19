/*
* SmarAct MCS2 programming example: Working with bitmasks
*
* The technique of masking bits in a options parameter plays a
* major role when working with the MCS2 API.
*
* Several properties are used to define a specific behaviour of a function.
* In general, properties which specify their value as bitfield of independent flags,
* controlling different features of a function, are named "Options". (Like the Referencing Options).
* (The Channel State property returns the state as bitfield too)
* Bit masking should be used to define option bitfields of these properties.
* If you are not familiar with this techniques, see also available information from the internet
* to get some more explanation, e.g. https://en.wikipedia.org/wiki/Mask_(computing).
*
* In contrast, properties which set a specific value out of a defined set
* of valid values are named "Mode".
* (Like the Sensor Power Mode property).
*
* For a full command reference see the MCS2 Programmers Guide.
*
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "SmarActControl.h"

SA_CTL_DeviceHandle_t dHandle;

void exitOnError(SA_CTL_Result_t result) {
    if (result != SA_CTL_ERROR_NONE) {
        SA_CTL_Close(dHandle);
        // Passing an error code to "SA_CTL_GetResultInfo" returns a human readable string
        // specifying the error.
        printf("MCS2 %s (error: 0x%04x).\nPress return to exit.\n",SA_CTL_GetResultInfo(result), result);
        getchar();
        exit(1);
    }
}

int main() {

    printf("************************************************************\n");
    printf("* SmarAct MCS2 Programming Example (Working with bitmasks) *\n");
    printf("************************************************************\n");

    // Read the version of the library
    // Note: this is the only function that does not require the library to be initialized.
    const char *version = SA_CTL_GetFullVersionString();
    printf("SmarActCTL library version: %s.\n", version);

    char locator[] = { "usb:ix:0" };
    char config[] = {""};
    SA_CTL_Result_t result;
    int8_t channel = 0;

    result = SA_CTL_Open(&dHandle, locator, config);
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to open \"%s\".\n",locator);
    }
    exitOnError(result);
    printf("MCS2 opened \"%s\".\n", locator);

    // ----------------------------------------------------------------------------------------------------------------
    // Setting explicit modes:
    // Properties which define a specific mode may be set directly to their value.
    // Here we set the sensor power mode to "SA_CTL_SENSOR_MODE_ENABLED".
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_SENSOR_POWER_MODE, SA_CTL_SENSOR_MODE_ENABLED);
    exitOnError(result);
    // ----------------------------------------------------------------------------------------------------------------
    // Reading a specific channel state flag:
    // -> Masking bits to "0"
    // Use bitwise AND to mask the channel state
    // to get only the information which is of interst. All other flags are set to zero and thus "ignored" for the check.

    // According to the rules:
    // Y AND 0 = 0
    // Y AND 1 = Y
    // one or more flags may be given to define the mask.
    // Here we only need the "sensorPresent" information, which is in bit 5 of the channel state.
    uint32_t channelState;
    result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_STATE, (int32_t*)&channelState, 0);
    exitOnError(result);
    bool sensorPresent = ((channelState & SA_CTL_CH_STATE_BIT_SENSOR_PRESENT) != 0);
    if (sensorPresent)
        printf("MCS2 channel %d has a sensor.\n", channel);
    // ----------------------------------------------------------------------------------------------------------------
    // Setting specific options flags:
    // -> Masking bits to "1"
    // Use bitwise OR to build the value of an options property.

    // According to the rules:
    // Y OR 1 = 1
    // Y OR 0 = Y
    // we combine the "startDirection" (Bit 0) and the "autoZero" (Bit 2) flags to define the referencing
    // behaviour.
    uint32_t refOptions = SA_CTL_REF_OPT_BIT_START_DIR | SA_CTL_REF_OPT_BIT_AUTO_ZERO;
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_REFERENCING_OPTIONS, refOptions);
    exitOnError(result);
    // ----------------------------------------------------------------------------------------------------------------
    // Clear a specific flag:
    // -> Read -> Modify -> Write

    // In case we only want to clear a specific flag without explicitly modifying other ones,
    // we must read the value, modify the interesting flag (using bitmasking) and write back the value of the property.

    // Just to show this, we disable the "autoZero" option of the (previously set) referencing options.
    // To mask the bit to zero we must use the inverted value of the "AutoZero" flag. This clears the bit 2 but keeps
    // all other flags at their previous value.
    // Here we use the "~" operator to invert the value.
    // 0x00000004 inverted -> 0xFFFFFFFB
    result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_REFERENCING_OPTIONS, (int32_t*)&refOptions, 0);
    exitOnError(result);
    refOptions = refOptions & (~SA_CTL_REF_OPT_BIT_AUTO_ZERO);
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_REFERENCING_OPTIONS, refOptions);
    exitOnError(result);
    // ----------------------------------------------------------------------------------------------------------------
    // Set a specific flag:
    // -> Read -> Modify -> Write

    // If we only want to set a specific flag without modifying other ones, we must read the value, modify the interesting
    // flag (using bitmasking) and write back the value of the property.

    // To show this, we again enable the "autoZero" referencing option.

    // To mask the bit to one we must OR bit 2 to the previously read value. This forces the "AutoZero" flag to one and
    // keeps all other flags at their previous value.
    result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_REFERENCING_OPTIONS, (int32_t*)&refOptions, 0);
    exitOnError(result);
    refOptions = refOptions | SA_CTL_REF_OPT_BIT_AUTO_ZERO;
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_REFERENCING_OPTIONS, refOptions);
    exitOnError(result);
    // ----------------------------------------------------------------------------------------------------------------
    // Before closing the program the connection to the device must be closed by calling "SA_CTL_Close".
    SA_CTL_Close(dHandle);
    printf("MCS2 close.\n");

    printf("*******************************************************\n");
    printf("Done. Press return to exit.\n");
    getchar();
    return 0;
}
