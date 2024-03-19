/*
* SmarAct MCS2 programming example: MoveLoop
*
* This programming example performs
* positioner movement in a loop and polls
* the channel state to determine the end
* of the movement.
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
    printf("*  SmarAct MCS2 Programming Example (MoveLoop)        *\n");
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

    // We assume that there is a linear positioner with integrated sensor connected to channel 0.
    int8_t channel = 0;
    int32_t type, state, mask;
    // Set move mode to relative movement.
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_RELATIVE);
    exitOnError(result);
    // Set move velocity to 2 mm/s.
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 2000000000);
    exitOnError(result);
    // Set move acceleration to 20 mm/s2.
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 20000000000);
    exitOnError(result);
    // Enable the amplifier.
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_AMPLIFIER_ENABLED, SA_CTL_TRUE);
    exitOnError(result);
    // Read the channel type to determine if additional configuration is required for this type of channel.
    result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_TYPE, &type, 0);
    exitOnError(result);

    // The following code performs one or two sequences of closed loop movement in a loop.
    // The channel state is polled to trigger the direction reverse.
    // Observe the different state flags used to determine the end of the movement.

    uint32_t noOfSequences = 2;
    if (type == SA_CTL_STICK_SLIP_PIEZO_DRIVER) {
        // The hold time specifies how long the position is actively held after reaching the target.
        // This can be useful to guarantee that a position is held precisely including
        // compensation of drift effects, etc.
        // We set the hold time to 1000 ms to notice the effect of the hold time (in the second sequence).
        result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_HOLD_TIME, 1000);
        exitOnError(result);
    } else if (type == SA_CTL_MAGNETIC_DRIVER) {
        // Magnetic driven positioners are always in the holding state when stopped. Therefore
        // there is no hold time property for this type of channel and we dont run the second sequence.
        noOfSequences = 1;
    }

    // FIRST SEQUENCE
    printf("*******************************************************\n");
    printf("First sequence:\n");
    printf("Reverse direction instantly after reaching the target position\n");
    printf("-> Press return to start the move loop.\n");
    getchar();

    int64_t position = 1000000000;  // 1 mm
    for (uint32_t sequ = 0; sequ < noOfSequences; sequ++) {
        for (uint32_t i = 0; i < 4; i++) {
            // Command alternating positive and negative movement in a loop.
            result = SA_CTL_Move(dHandle, channel, position, 0);
            exitOnError(result);
            printf("MCS2 move...\n");
            // Poll the channel state to wait for the movement to finish.
            while (1) {
                if (sequ == 0) {
                    // The first sequence waits for the "SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING" state flag to be read as zero
                    // and reverses the movement direction instantly.
                    // The "SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING" flag remains set until the target position was reached.
                    // (or an endstop was detected)
                    mask = SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING;
                } else {
                    // The second sequence additionally checks the "SA_CTL_CH_STATE_BIT_CLOSED_LOOP_ACTIVE" flag to wait
                    // for the hold time to elapse.
                    mask = (SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING | SA_CTL_CH_STATE_BIT_CLOSED_LOOP_ACTIVE);
                }
                // Check if the movement was successful.
                result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
                exitOnError(result);
                // The SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED channel state flag indicates a failed movement.
                if ((state & SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED) != 0) {
                    // The channel error property may then be read to determine the reason of the error.
                    int32_t error;
                    result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_ERROR, &error, 0);
                    exitOnError(result);
                    printf("MCS2 movement failed: %s (error: 0x%04x), abort.\n", SA_CTL_GetResultInfo(error), error);
                    break;
                    // Alternatively, the following channel state flags may be tested to determine the reason
                    // of a failed movement:
                    // - SA_CTL_CH_STATE_BIT_END_STOP_REACHED
                    // - SA_CTL_CH_STATE_BIT_RANGE_LIMIT_REACHED
                    // - SA_CTL_CH_STATE_BIT_FOLLOWING_LIMIT_REACHED
                    // See the MCS2 Programmers Guide for more information on the specific channel state flags.
                }
                if ((state & mask) == 0) {
                    if (i < 3) printf("MCS2 reverse direction [%d].\n", i+1);
                    else printf("MCS2 move loop done.\n");
                    position = -position;
                    break;
                }
            }
        }
        if ((noOfSequences == 2) && (sequ == 0)) {
            // SECOND SEQUENCE (optional)
            printf("*******************************************************\n");
            printf("Second sequence:\n");
            printf("Reverse direction after reaching the target position\n");
            printf("AND hold time elapsed\n");
            printf("-> Press return to start the move loop.\n");
            getchar();
        }
    }

    printf("Done. Press return to exit.\n");
    getchar();

    // Before closing the program the connection to the device must be closed by calling "SA_CTL_Close".
    SA_CTL_Close(dHandle);
    printf("MCS2 close.\n");
    printf("*******************************************************\n");
    return 0;
}