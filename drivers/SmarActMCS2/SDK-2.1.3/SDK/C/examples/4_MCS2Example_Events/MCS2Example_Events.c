/*
* SmarAct MCS2 programming example: Events
*
* This programming example shows you how to
* receive events from an MCS2 device.
*
* For a full command reference see the MCS2 Programmers Guide.
*
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "SmarActControl.h"

#if defined(_WIN32)
#define strtok_r strtok_s
#endif

SA_CTL_DeviceHandle_t dHandle;

SA_CTL_Result_t waitForEvent(void);

void exitOnError(SA_CTL_Result_t result) {
    if (result != SA_CTL_ERROR_NONE) {
        SA_CTL_Close(dHandle);
        // Passing an error code to "SA_CTL_GetResultInfo" returns a human readable string
        // specifying the error.
        printf("MCS2 %s (error: 0x%04x).\nPress return to exit.\n", SA_CTL_GetResultInfo(result), result);
        getchar();
        exit(1);
    }
}

int main() {

    SA_CTL_Result_t result;

    printf("*******************************************************\n");
    printf("*  SmarAct MCS2 Programming Example (Events)          *\n");
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

    // Events are asynchronously generated notifications from the controller.
    // They can be used to receive state information from the controller
    // without continuously polling the state.
    // See the MCS2 Programmers Guide for a list of all available events.

    // The following code commands a closed-loop relative movement and uses the
    // "SA_CTL_WaitForEvent" function to receive events from the controller.

    int8_t channel = 0;

    // Set move mode to closed-loop relative movement.
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_RELATIVE);
    exitOnError(result);
    // Set move velocity [in pm/s].
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 100000000);
    exitOnError(result);
    // Set move acceleration [in pm/s2].
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 10000000000);
    exitOnError(result);
    // Enable the amplifier.
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_AMPLIFIER_ENABLED, SA_CTL_TRUE);
    exitOnError(result);

    printf("*******************************************************\n");
    printf("-> Press return to start movement.\n");
    getchar();
    // Command a closed-loop movement.
    result = SA_CTL_Move(dHandle, channel, 100000000, 0);
    exitOnError(result);
    // Wait for the "SA_CTL_EVENT_MOVEMENT_FINISHED" event.
    waitForEvent();

    // Again commanding a (time consuming) movement. Intentionally provoking a timeout.
    printf("-> Press return to start movement.\n");
    getchar();
    result = SA_CTL_Move(dHandle, channel, 500000000, 0);
    exitOnError(result);
    // Will return with a timeout.
    waitForEvent();

    // Stop positioner.
    result = SA_CTL_Stop(dHandle, channel, 0);
    printf("Stop channel %d.\n", channel);
    exitOnError(result);

    printf("Done. Press return to exit.\n");
    getchar();

    // Before closing the program the connection to the device must be closed by calling "SA_CTL_Close".
    SA_CTL_Close(dHandle);
    printf("MCS2 close.\n");
    printf("*******************************************************\n");
    return 0;
}

SA_CTL_Result_t waitForEvent(void) {
    SA_CTL_Result_t result;
    SA_CTL_Event_t event;
    // The wait for event function blocks until an event was received or the timeout elapsed.
    // In case of timeout, the function returns with "SA_CTL_ERROR_TIMEOUT".
    // If the "timeout" parameter is set to "SA_CTL_INFINITE" the call blocks until an event is received.
    // This can be useful in case the SA_CTL_WaitForEvent function runs in a separate thread.
    // For simplicity, this is not shown here thus we set a timeout of 3 seconds.
    int32_t timeout = 3000; // in ms
    result = SA_CTL_WaitForEvent(dHandle, &event, timeout);
    if (result == SA_CTL_ERROR_TIMEOUT) {
        printf("MCS2 wait for event timed out after %d ms\n", timeout);
    } else if (result == SA_CTL_ERROR_NONE) {
        // The "type" field specifies the event.
        // The "idx" field holds the channel where the event came from.
        // The "i32" data field gives additional information about the event, e.g. error code.
        // Passing the event to "SA_CTL_GetEventInfo" returns a human readable string
        // specifying the event.
        switch (event.type) {
            case SA_CTL_EVENT_MOVEMENT_FINISHED:
                // Movement finished.
                if (event.i32 == SA_CTL_ERROR_NONE) printf("MCS2 movement finished, channel: %d\n", event.idx);
                // The movement failed for some reason. E.g. an endstop was detected.
                else printf("MCS2 movement finished, channel: %d, error: 0x%04x (%s)\n ", event.idx, event.i32, SA_CTL_GetResultInfo(event.i32));
                break;
            default:
                // The code should be prepared to handle unexpected events beside the expected ones.
                printf("MCS2 received event: %s",SA_CTL_GetEventInfo(&event));
                break;
            }
    }
    return result;
}