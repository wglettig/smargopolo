/*
* SmarAct MCS2 programming example: Device-Info
*
* This programming example shows how to read device,
* module and channel properties to query various information
* about the device.
*
* For a full command reference see the MCS2 Programmers Guide.
*
*/

#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#include <stdbool.h>
#include <string.h>
#include "SmarActControl.h"

#if defined(_WIN32)
#define strtok_r strtok_s
#endif

SA_CTL_DeviceHandle_t dHandle;

void exitOnError(SA_CTL_Result_t result) {
    if (result != SA_CTL_ERROR_NONE) {
        SA_CTL_Close(dHandle);
        // Passing an error code to "SA_CTL_GetResultInfo"
        // returns a human readable string specifying the error.
        printf("MCS2 %s (error: 0x%04x).\nPress return to exit.\n",SA_CTL_GetResultInfo(result), result);
        _getch();
        exit(1);
    }
}

int main() {

    SA_CTL_Result_t result;

    printf("*******************************************************\n");
    printf("*  SmarAct MCS2 Programming Example (Device Info)     *\n");
    printf("*******************************************************\n");
    // Read the version of the library
    // Note: this is the only function that does not require the library to be initialized.
    const char *version = SA_CTL_GetFullVersionString();
    printf("SmarActCTL library version: %s.\n", version);

    // Find available MCS2 devices
    char deviceList[1024];
    size_t ioDeviceListLen = sizeof(deviceList);
    result = SA_CTL_FindDevices("", deviceList, &ioDeviceListLen);
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to find devices.\n");
    }
    exitOnError(result);
    if (strlen(deviceList) == 0) {
        printf("MCS2 no devices found. Exit.\n");
        getchar();
        exit(1);
    }
    else {
        printf("MCS2 available devices:\n%s\n", deviceList);
    }
    // Open the first MCS2 device from the list
    char *ptr;
    strtok_r(deviceList, "\n", &ptr);
    char *locator = deviceList;
    result = SA_CTL_Open(&dHandle, locator, "");
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to open \"%s\".\n", locator);
    }
    exitOnError(result);
    printf("MCS2 opened \"%s\".\n", locator);
    
    // Note on properties:
    // Not all properties are applicable for all driver modules and their channels.
    // Refer to the Property Reference of the MCS2 Programmers Guide to determine if
    // a property is valid for a specific module. Reading or writing a property which
    // is not available returns a SA_CTL_ERROR_INVALID_KEY error.
    // Read the "Interface Type", "Module Type" or "Channel Type" properties to determine 
    // the type of the interface, module or channel.
    
    // Read device info

    // Note for device properties:
    // The controller is already addressed by the device handle. Therefore,
    // the index parameter is unused and must be set to zero.
    int32_t type, state, noOfBusModules, noOfChannels, num;
    char buf[SA_CTL_STRING_MAX_LENGTH];
    size_t ioStringSize = sizeof(buf);
    result = SA_CTL_GetProperty_s(dHandle, 0, SA_CTL_PKEY_DEVICE_SERIAL_NUMBER, buf, &ioStringSize);
    exitOnError(result);
    printf("\nDevice Serial Number: %s\n", buf);
    ioStringSize = sizeof(buf);
    result = SA_CTL_GetProperty_s(dHandle, 0, SA_CTL_PKEY_DEVICE_NAME, buf, &ioStringSize);
    exitOnError(result);
    printf("Device Name: %s\n", buf);
    result = SA_CTL_GetProperty_i32(dHandle, 0, SA_CTL_PKEY_INTERFACE_TYPE, &type, 0);
    exitOnError(result);
    if (type == SA_CTL_INTERFACE_USB) printf("Interface Type: USB\n");
    else if (type == SA_CTL_INTERFACE_ETHERNET) printf("Interface Type: Ethernet\n");
    result = SA_CTL_GetProperty_i32(dHandle, 0, SA_CTL_PKEY_DEVICE_STATE, &state, 0);
    exitOnError(result);
    printf("Hand Control Module present: ");
    if ((state & SA_CTL_DEV_STATE_BIT_HM_PRESENT) != 0) printf("yes\n"); else printf("no\n"); 
    // Use bitwise AND to mask the channel/module state to get only the information
    // which is of interst. All other flags are set to zero and thus "ignored" for the check.
    result = SA_CTL_GetProperty_i32(dHandle, 0, SA_CTL_PKEY_NUMBER_OF_BUS_MODULES, &noOfBusModules, 0);
    exitOnError(result);
    printf("Number of Bus Modules: %d\n", noOfBusModules);
    result = SA_CTL_GetProperty_i32(dHandle, 0, SA_CTL_PKEY_NUMBER_OF_CHANNELS, &noOfChannels, 0);
    exitOnError(result);
    printf("Total Number of Channels: %d\n", noOfChannels);
    printf("*******************************************************\n");
    // Read module info
    // The "moduleIndex" must be passed to all module properties. Note that it is zero-based.
    for (uint8_t i=0; i<noOfBusModules; ++i) {
        result = SA_CTL_GetProperty_i32(dHandle, i, SA_CTL_PKEY_MODULE_TYPE, &type, 0);
        exitOnError(result);
        printf("Module %d", i);
        if (type == SA_CTL_STICK_SLIP_PIEZO_DRIVER) printf(" (Stick-Slip-Piezo-Driver):\n");
        else if (type == SA_CTL_MAGNETIC_DRIVER) printf(" (Magnetic-Driver):\n");
        result = SA_CTL_GetProperty_i32(dHandle, i, SA_CTL_PKEY_NUMBER_OF_BUS_MODULE_CHANNELS, &num, 0);
        exitOnError(result);
        printf("    Number of Bus Module Channels: %d\n", num);
        result = SA_CTL_GetProperty_i32(dHandle, i, SA_CTL_PKEY_MODULE_STATE, &state, 0);
        exitOnError(result);
        printf("    Sensor Module present: ");
        if ((state & SA_CTL_MOD_STATE_BIT_SM_PRESENT) != 0) printf("yes\n"); else printf("no\n");
    }
    printf("-------------------------------------------------------\n");
    // Read channel info
    // The "channelIndex" must be passed to all channel properties. Note that it is zero-based.
    for (uint8_t i=0; i<noOfChannels; ++i) {
        printf("        Channel: %d\n", i);
        ioStringSize = sizeof(buf);
        result = SA_CTL_GetProperty_s(dHandle, i, SA_CTL_PKEY_POSITIONER_TYPE_NAME, buf, &ioStringSize);
        exitOnError(result);
        result = SA_CTL_GetProperty_i32(dHandle, i, SA_CTL_PKEY_POSITIONER_TYPE, &type, 0);
        exitOnError(result);
        printf("        Positioner Type: %s (%d)\n", buf, type);
        result = SA_CTL_GetProperty_i32(dHandle, i, SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
        exitOnError(result);
        printf("        Amplifier enabled: ");
        if ((state & SA_CTL_CH_STATE_BIT_AMPLIFIER_ENABLED) != 0) printf("yes\n"); else printf("no\n");
        // Here we read the "Channel Type" property to determine which additional properties and flags are of interest.
        // E.g. the maxCLF property is only available for Stick-Slip-Driver channels and the "isPhased"
        // Channel State flag is only available for Magnetic-Driver channels.
        // Note that the "Module Type" and "Channel Type" properties share the same list of types.
        result = SA_CTL_GetProperty_i32(dHandle, i, SA_CTL_PKEY_CHANNEL_TYPE, &type, 0);
        exitOnError(result);
        if (type == SA_CTL_STICK_SLIP_PIEZO_DRIVER) {
            int32_t maxCLF;
            result = SA_CTL_GetProperty_i32(dHandle, i, SA_CTL_PKEY_MAX_CL_FREQUENCY, &maxCLF, 0);
            exitOnError(result);
            printf("        Max-CLF: %d Hz\n", maxCLF);
        } else if (type == SA_CTL_MAGNETIC_DRIVER) {
            printf("        Channel is phased: ");
            if ((state & SA_CTL_CH_STATE_BIT_IS_PHASED) != 0) printf("yes\n"); else printf("no\n");
        }
        printf("-------------------------------------------------------\n");
    }
    printf("Done. Press any key to exit.\n");
    _getch();

    // Before closing the program the connection to the device must be closed by calling "SA_CTL_Close".
    SA_CTL_Close(dHandle);
    printf("MCS2 close.\n");
    printf("*******************************************************\n");
    return 0;
}
