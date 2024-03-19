/*
* SmarAct MCS2 programming example: MagneticDriver
*
* This programming example demonstrates how to control
* electromagnetic driven positioners with the MCS2
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

void waitWhile(int8_t channel, uint32_t mask) {
    uint32_t state;
    while (1) {
        SA_CTL_Result_t result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_STATE, (int32_t*)&state, 0);
        exitOnError(result);
        if ((state & mask) == 0) break;
    }
}

int main() {

    SA_CTL_Result_t result;

    printf("***************************************************************\n");
    printf("*  SmarAct MCS2 Programming Example (Electromagnetic Driver)  *\n");
    printf("***************************************************************\n");

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

    int8_t channel = 0;
    int32_t type, state, error;

    // Verify that there is an electromagnetic driven positioner available.
    result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_TYPE, &type, 0);
    exitOnError(result);
    result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
    exitOnError(result);
    if (type != SA_CTL_MAGNETIC_DRIVER) {
        printf("MCS2 channel %d is not an electromagnetic driver channel, abort.\n", channel);
        SA_CTL_Close(dHandle);
        getchar();
        exit(1);
    } else if ((state & SA_CTL_CH_STATE_BIT_SENSOR_PRESENT) == 0) {
        printf("MCS2 no positioner connected to channel %d, abort.\n", channel);
        SA_CTL_Close(dHandle);
        getchar();
        exit(1);
    }

    // The amplifier of electromagnetic driver channels is disabled at startup and must be explicitly enabled
    // before being able to perform closed-loop movements. This also implicitly starts the phasing sequence of
    // the positioner if it is not already phased.
    // The SA_CTL_CH_STATE_BIT_AMPLIFIER_ENABLED channel state bit reflects the state of the amplifier.
    if ((state & SA_CTL_CH_STATE_BIT_AMPLIFIER_ENABLED) == 0) {
        printf("*******************************************************\n");
        printf("-> Press return to enable the amplifier (and start the phasing sequence).\n");
        getchar();
        result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_AMPLIFIER_ENABLED, SA_CTL_TRUE);
        exitOnError(result);
        if ((state & SA_CTL_CH_STATE_BIT_IS_PHASED) == 0) {
            printf("MCS2 phasing...\n");
            // Note that no external force or displacement must be applied to the positioner while the sequence is running.
            // The phasing takes some time to complete. During this time the SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING
            // channel state bit is set.
            // The channel generates a SA_CTL_EVENT_PHASING_FINISHED event once the sequence has finished.
            // In this example we poll the channel state to determine the end of the phasing sequence instead of
            // listening to events.
            waitWhile(channel, SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING);
            // The SA_CTL_CH_STATE_BIT_IS_PHASED channel state flag should then be checked to verify that the
            // phasing sequence finished successfully.
            result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
            exitOnError(result);
            if ((state & SA_CTL_CH_STATE_BIT_IS_PHASED) == 0) {
                printf("MCS2 could not establish the phasing reference for channel %d, abort.\n", channel);
                SA_CTL_Close(dHandle);
                getchar();
                exit(1);
            } else {
                printf("MCS2 phase reference found.\n");
            }
        }
    }
    // Perform referencing sequence if physical position is not known
    if ((state & SA_CTL_CH_STATE_BIT_IS_REFERENCED) == 0) {
        // Since the physical scale is not known yet no range limits can be used here.
        // Make sure to use a moderate move velocity for the referencing sequence.
        // Disable software range limits while referencing.
        result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_RANGE_LIMIT_MIN, 0);
        exitOnError(result);
        result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_RANGE_LIMIT_MAX, 0);
        exitOnError(result);
        result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_REFERENCING_OPTIONS, 0);
        exitOnError(result);
        // The move velocity and acceleration properties also define the parameters for the referencing.
        result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 2000000000);
        exitOnError(result);
        result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 100000000000);
        exitOnError(result);
        printf("*******************************************************\n");
        printf("-> Press return to start the referencing.\n");
        getchar();
        printf("MCS2 referencing...\n");
        result = SA_CTL_Reference(dHandle, channel, 0);
        exitOnError(result);
        waitWhile(channel, SA_CTL_CH_STATE_BIT_REFERENCING);
        // Check if the referencing was successful.
        result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
        exitOnError(result);
        // The SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED indicates a failed referencing.
        if ((state & SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED) != 0) {
            // The channel error property may then be read to determine the reason of the error.
            result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_ERROR, &error, 0);
            exitOnError(result);
            printf("MCS2 referencing failed: %s (error: 0x%04x), abort.\n", SA_CTL_GetResultInfo(error), error);
            SA_CTL_Close(dHandle);
            getchar();
            exit(1);
        } else {
            printf("MCS2 reference found.\n");
        }
    }
    // Note that electromagnetic driven positioners can reach very high velocities!
    // Moving into a physical endstop with high velocity can damage the positioner.
    // Therefore it is recommended to use the software range limits to define the active movement
    // range of the positioner. When commanding the positioner towards a limit the positioner is
    // decelerated to zero velocity in a way that it comes to a halt on the specified limit position.
    // Note: adjust this limits to match the movement range of your positioner!
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_RANGE_LIMIT_MIN, -20000000000);
    exitOnError(result);
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_RANGE_LIMIT_MAX, 20000000000);
    exitOnError(result);

    printf("*******************************************************\n");
    printf("-> Press return to start the movement.\n");
    getchar();
    // Set move mode to absolute movement.
    // Note: Open-loop movement is not available for electromagnetic driver channels.
    // (only the SA_CTL_MOVE_MODE_CL_ABSOLUTE and SA_CTL_MOVE_MODE_CL_RELATIVE move modes are valid)
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
    exitOnError(result);
    // Set move velocity and acceleration.
    // Note that the velocity and acceleration control must be used for all movements. A velocity / acceleration
    // value of 0 (to disable the velocity / acceleration control) is invalid for magnetic driver channels.
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 10000000000);
    exitOnError(result);
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 10000000000);
    exitOnError(result);

    int64_t positions[] = {2000000000,-2000000000,10000000000,-10000000000}; // in pm
    for (uint32_t i = 0; i < 4; i++) {
        printf("MCS2 move channel %d to position: %" PRId64"...\n", channel, positions[i]);
        result = SA_CTL_Move(dHandle, channel, positions[i], 0);
        exitOnError(result);
        waitWhile(channel, SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING);
        // Check if the movement was successful.
        result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
        exitOnError(result);
        // The SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED channel state flag indicates a failed movement.
        if ((state & SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED) != 0) {
            // The channel error property may then be read to determine the reason of the error.
            result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_ERROR, &error, 0);
            exitOnError(result);
            printf("MCS2 movement failed: %s (error: 0x%04x).\n", SA_CTL_GetResultInfo(error), error);
            break;
            // Alternatively, the following channel state flags may be tested to determine the reason
            // of a failed movement:
            // - SA_CTL_CH_STATE_BIT_END_STOP_REACHED
            // - SA_CTL_CH_STATE_BIT_RANGE_LIMIT_REACHED
            // - SA_CTL_CH_STATE_BIT_FOLLOWING_LIMIT_REACHED
            // - SA_CTL_CH_STATE_BIT_POSITIONER_OVERLOAD
            // - SA_CTL_CH_STATE_BIT_POSITIONER_FAULT
            // See the MCS2 Programmers Guide for more information on the specific channel state flags.
        }
        // The electromagnetic driver monitors the output current of each channel to detect an overload condition
        // of the positioner. This prevents thermal overheating and potential damage of the positioners coils,
        // isolation and permanent magnets.
        // If an over load is detected the control-loop is disabled to protect the positioner.
        // WARNING
        // Electromagnetic driven positioners are not self-locking. Disabling the control-loop removes
        // any holding force from the positioner. Make sure not to damage any equipment
        // when the positioner unintentionally changes its position!

        // The present load level may be read in percent with the motor load property. This may be useful
        // to estimate the motor load while performing movements before the overload protection trips and
        // disables the control-loop. In case the motor load reaches a level close to 100 % the number of
        // movements per time, the movement acceleration and/or the mechanical load attached to the
        // positioner should be reduced.
        int32_t load;
        result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOTOR_LOAD, &load, 0);
        exitOnError(result);
        printf("MCS2 short-term motor over load: %d %%\n", load);
    }
    // Disable the amplifier again.
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_AMPLIFIER_ENABLED, SA_CTL_FALSE);
    exitOnError(result);
    printf("Done. Press return to exit.\n");
    getchar();

    // Before closing the program the connection to the device must be closed by calling "SA_CTL_Close".
    SA_CTL_Close(dHandle);
    printf("MCS2 close.\n");
    printf("*******************************************************\n");
    return 0;
}
