/*
* SmarAct MCS2 programming example: Command Groups
*
* This programming example shows how to use command groups
* to atomic group MCS2 properties and commands.
*
* For a full command reference see the MCS2 Programmers Guide.
*
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
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

void infoOnError(SA_CTL_Result_t result) {
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 %s (error: 0x%04x).\n", SA_CTL_GetResultInfo(result), result);
    }
}

int main() {

    SA_CTL_Result_t result;

    printf("*******************************************************\n");
    printf("*  SmarAct MCS2 Programming Example (Command Groups)  *\n");
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

    // Command groups offer the possibility to define an atomic group of commands that is executed
    // synchronously. In addition, a command group may not only be triggered via software,
    // but alternatively via an external trigger.
    // Several properties and movement commands can be set atomic for one or more channels.
    //
    // A typical set of properties and commands may be:
    // - set property : move mode
    // - set property : move velocity
    // - move
    // To define a command group simply surround the commands that should be grouped with calls
    // to the SA_CTL_OpenCommandGroup and SA_CTL_CloseCommandGroup functions and pass
    // the transmit handle received from the SA_CTL_OpenCommandGroup function to all commands
    // to be grouped.
    //
    // Events must be used to get status information in the context of command groups.
    // A SA_CTL_EVENT_CMD_GROUP_TRIGGERED event notifies the software that the grouped commands
    // are processed.
    // Note that movement commands do not generate a result packet. Instead a
    // SA_CTL_EVENT_MOVEMENT_FINISHED event is generated once the command finished.

    // The following code configures two channels for a closed loop movement and then moves both
    // channels to some target position. The grouped commands and properties are treated as one command
    // and the movement of both channels start synchronously.
    // (in this case as soon as the command group is closed, since the direct trigger mode is used).

    SA_CTL_TransmitHandle_t txHandle;
    SA_CTL_RequestID_t rId[6];

    // First enable the amplifiers of both channels
    printf("*******************************************************\n");
    printf("-> Press return to start command group (amplifier enable).\n");
    getchar();
    result = SA_CTL_OpenCommandGroup(dHandle, &txHandle, SA_CTL_CMD_GROUP_TRIGGER_MODE_DIRECT);
    exitOnError(result);
    result = SA_CTL_RequestWriteProperty_i32(dHandle, 0, SA_CTL_PKEY_AMPLIFIER_ENABLED,
                                             SA_CTL_TRUE,
                                             &rId[0], txHandle);
    exitOnError(result);
    result = SA_CTL_RequestWriteProperty_i32(dHandle, 1, SA_CTL_PKEY_AMPLIFIER_ENABLED,
                                             SA_CTL_TRUE,
                                             &rId[1], txHandle);
    exitOnError(result);
    result = SA_CTL_CloseCommandGroup(dHandle, txHandle);
    exitOnError(result);
    // Wait for the "triggered" event before reading the results
    waitForEvent();
    // One important thing to notice is that the SA_CTL_WaitForWrite function calls must be issued
    // after the command group was closed. Otherwise the function calls will block.
    // Note that synchronous property accesses cannot be put into a command group.
    result = SA_CTL_WaitForWrite(dHandle, rId[0]);
    infoOnError(result);
    result = SA_CTL_WaitForWrite(dHandle, rId[1]);
    infoOnError(result);

    printf("*******************************************************\n");
    printf("-> Press return to start command group (movement).\n");
    getchar();

    result = SA_CTL_OpenCommandGroup(dHandle, &txHandle, SA_CTL_CMD_GROUP_TRIGGER_MODE_DIRECT);
    exitOnError(result);
    result = SA_CTL_RequestWriteProperty_i32(dHandle, 0, SA_CTL_PKEY_MOVE_MODE,
                                             SA_CTL_MOVE_MODE_CL_ABSOLUTE,
                                             &rId[0], txHandle);
    exitOnError(result);
    result = SA_CTL_RequestWriteProperty_i32(dHandle, 1, SA_CTL_PKEY_MOVE_MODE,
                                             SA_CTL_MOVE_MODE_CL_ABSOLUTE,
                                             &rId[1], txHandle);
    exitOnError(result);
    result = SA_CTL_RequestWriteProperty_i64(dHandle, 0, SA_CTL_PKEY_MOVE_VELOCITY,
                                             1000000000,
                                             &rId[2], txHandle);
    exitOnError(result);
    result = SA_CTL_RequestWriteProperty_i64(dHandle, 1, SA_CTL_PKEY_MOVE_VELOCITY,
                                             1000000000,
                                             &rId[3], txHandle);
    exitOnError(result);
    result = SA_CTL_RequestWriteProperty_i64(dHandle, 0, SA_CTL_PKEY_MOVE_ACCELERATION,
                                             10000000000,
                                             &rId[4], txHandle);
    exitOnError(result);
    result = SA_CTL_RequestWriteProperty_i64(dHandle, 1, SA_CTL_PKEY_MOVE_ACCELERATION,
                                             10000000000,
                                             &rId[5], txHandle);
    exitOnError(result);
    result = SA_CTL_Move(dHandle, 0, 1000000000, txHandle);
    exitOnError(result);
    result = SA_CTL_Move(dHandle, 1, 2000000000, txHandle);
    exitOnError(result);
    result = SA_CTL_CloseCommandGroup(dHandle, txHandle);
    exitOnError(result);
    // Wait for the "triggered" event before reading the results
    waitForEvent();
    result = SA_CTL_WaitForWrite(dHandle, rId[0]);
    infoOnError(result);
    result = SA_CTL_WaitForWrite(dHandle, rId[1]);
    infoOnError(result);
    result = SA_CTL_WaitForWrite(dHandle, rId[2]);
    infoOnError(result);
    result = SA_CTL_WaitForWrite(dHandle, rId[3]);
    infoOnError(result);
    result = SA_CTL_WaitForWrite(dHandle, rId[4]);
    infoOnError(result);
    result = SA_CTL_WaitForWrite(dHandle, rId[5]);
    infoOnError(result);

    printf("*******************************************************\n");
    printf("-> Press return to read the position.\n");
    getchar();

    // Next we create a command group to read some properties: the current position of both channels.
    result = SA_CTL_OpenCommandGroup(dHandle, &txHandle, SA_CTL_CMD_GROUP_TRIGGER_MODE_DIRECT);
    exitOnError(result);
    result = SA_CTL_RequestReadProperty(dHandle, 0, SA_CTL_PKEY_POSITION, &rId[0], txHandle);
    exitOnError(result);
    result = SA_CTL_RequestReadProperty(dHandle, 1, SA_CTL_PKEY_POSITION, &rId[1], txHandle);
    exitOnError(result);
    result = SA_CTL_CloseCommandGroup(dHandle, txHandle);

    // Wait for the "triggered" event before reading the results
    waitForEvent();
    // The same rule applies as for write properties:
    // Put the SA_CTL_RequestReadProperty calls into the command group,
    // but issue e.g. SA_CTL_ReadProperty_i64 calls after the group close.
    int64_t position[2];
    result = SA_CTL_ReadProperty_i64(dHandle, rId[0], &position[0], 0);
    infoOnError(result);
    result = SA_CTL_ReadProperty_i64(dHandle, rId[1], &position[1], 0);
    infoOnError(result);

    printf("Position channel 0: %" PRId64" pm, channel 1: %" PRId64" pm.\n", position[0], position[1]);
    printf("Done. Press return to exit.\n");
    getchar();

    // Before closing the program the connection to the device must be closed by calling "SA_CTL_Close".
    SA_CTL_Close(dHandle);
    printf("MCS2 close.\n");
    printf("*******************************************************\n");
    return 0;
}

SA_CTL_Result_t waitForEvent(void) {
    SA_CTL_Result_t result, resultCode;
    SA_CTL_TransmitHandle_t txHandle;
    SA_CTL_Event_t event;
    // The wait for event function blocks until an event was received or the timeout elapsed.
    // In case of timeout, the function returns with "SA_CTL_ERROR_TIMEOUT".
    // For this example we are only interested in "SA_CTL_EVENT_CMD_GROUP_TRIGGERED" events
    // thus we drop all other events like "SA_CTL_EVENT_MOVEMENT_FINISHED" etc.
    int32_t timeout = 10000; // in ms
    bool done = false;
    while (!done) {
        result = SA_CTL_WaitForEvent(dHandle, &event, timeout);
        if (result == SA_CTL_ERROR_TIMEOUT) {
            printf("MCS2 wait for event timed out after %d ms\n", timeout);
            done = true;
        } else if (result == SA_CTL_ERROR_NONE) {
            switch (event.type) {
                // The "type" field specifies the event.
                // The "idx" field holds the device index for this event, it will always be "0", thus might be ignored here.
                // The "i32" data field gives additional information about the event.
                case SA_CTL_EVENT_CMD_GROUP_TRIGGERED:
                    // A command group has been executed.
                    // The event parameter holds:
                    // - the result code of the group (Bit 0-15)
                    // - the corresponding transmit handle of the group (Bit 31-24)
                    txHandle = SA_CTL_EVENT_PARAM_HANDLE(event.i32);
                    resultCode = SA_CTL_EVENT_PARAM_RESULT(event.i32);
                    if (resultCode == SA_CTL_ERROR_NONE) {
                        printf("MCS2 command group triggered, handle: %d\n", txHandle);
                    } else {
                        // The command group failed -> the reason may be found in the result code.
                        // To determine which command caused the error, read the individual results of the command
                        // with "SA_CTL_WaitForWrite" / "SA_CTL_ReadProperty_x".
                        printf("MCS2 command group failed, handle: %d, error: 0x%04x (%s)\n ", txHandle, resultCode, SA_CTL_GetResultInfo(resultCode));
                    }
                    done = true;
                    break;
                default:
                    // ignore other events and wait for the next one
                    break;
            }
        }
    }
    return result;
}
