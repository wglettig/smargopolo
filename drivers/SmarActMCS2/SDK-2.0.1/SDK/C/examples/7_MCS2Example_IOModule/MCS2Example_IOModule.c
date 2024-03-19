/*
* SmarAct MCS2 programming example: IO-Module
*
* This programming example shows how to configure a SmarAct MCS2 IO Module
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
// We assume that there is a linear positioner with integrated sensor connected to channel 0.
int8_t channel = 0;

void exitOnError(SA_CTL_Result_t result) {
    if (result != SA_CTL_ERROR_NONE) {
        SA_CTL_Close(dHandle);
        // Passing an error code to "SA_CTL_GetResultInfo" returns a human readable string
        // specifying the error.
        printf("MCS2 %s (error: 0x%04x).\nPress return to exit.\n",SA_CTL_GetResultInfo(result), result);
        _getch();
        exit(1);
    }
}

void waitForEvent(void) {
    SA_CTL_Event_t event;
    bool done = false;
    while (!done) {
        SA_CTL_Result_t result;
        uint32_t timeout = 10000;
        result = SA_CTL_WaitForEvent(dHandle, &event, timeout);
        // we use a timeout of 10s, so the function call returns
        // if an event was received or at the latest after 10 seconds
        if (result == SA_CTL_ERROR_TIMEOUT) {
            printf("MCS2 wait for event timed out after %d seconds.\nStop positioner\n", timeout/1000);
            result = SA_CTL_Stop(dHandle, channel, 0);
            exitOnError(result);
            return;
        }
        else if (result == SA_CTL_ERROR_NONE) {
            // The "type" field specifies the event.
            // The "idx" field holds the channel where the event came from.
            // The "i32" data field gives additional information about the event, e.g. error code.
            // Passing the event to "SA_CTL_GetEventInfo" returns a human readable string
            // specifying the event.
            switch (event.type) {
            case SA_CTL_EVENT_EMERGENCY_STOP_TRIGGERED:
                printf("MCS2 received event: %s\n", SA_CTL_GetEventInfo(&event));
                done = true;
                break;
            default:
                // The code should be prepared to handle unexpected events beside the expected ones.
                printf("MCS2 received unexpected event: %s\n", SA_CTL_GetEventInfo(&event));
                break;
            }
        } else {
            printf("MCS2 wait for event failed: %s\n", SA_CTL_GetResultInfo(result));
            return;
        }
    }
    return;
}

int main() {

    SA_CTL_Result_t result;

    printf("*******************************************************\n");
    printf("*  SmarAct MCS2 Programming Example (IO Module)       *\n");
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

    // Enable the amplifier
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_AMPLIFIER_ENABLED, SA_CTL_TRUE);

    // Please note:
    // The MCS2 must be equipped with an I/O module in order to execute the following examples.
    // If no I/O module is available the properties return a SA_CTL_ERROR_NO_IOM_PRESENT error.

    // Start the examples
    for (uint32_t sequ = 0; sequ < 2; sequ++) {
        char ret = '#';
        if (sequ == 0) {
            ret = '#';
            printf("*******************************************************\n");
            printf("First sequence:\n");
            printf("- Digital input triggers an emergency stop -\n");
            printf("-> press key to [c]ontinue or [s]kip.\n");
            while (!(ret == 'c' || ret == 's')) {
                ret = _getch();
            }
            if (ret == 'c') {
                // Device input trigger -> emergency stop:
                // We need to set the following device properties to configure the emergency stop input trigger:
                // 1. the emergency stop mode (behavior of the emergency stop function)
                // 2. the device input trigger mode (connect digital device input to the emergency stop function)
                // 3. the polarity of the device input signal (trigger emergency stop on rising / falling edge of the digital input signal)
                // Afterwards we start a slow, long movement which can be aborted by applying a digital signal to the input.
                // The trigger signal must be fed into the digital device input of an I/O module to stop all channels.

                // Note that the device input trigger is dedicated to the whole device and not to a specific channel.
                // Channel-based emergency stop is available by using the general-purpose digital inputs of an I/O module.
                // (Not the scope of this example.)

                printf("Set emergency stop mode to normal mode.\n");
                result = SA_CTL_SetProperty_i32(dHandle, 0, SA_CTL_PKEY_EMERGENCY_STOP_MODE, SA_CTL_EMERGENCY_STOP_MODE_NORMAL);
                exitOnError(result);
                printf("Set trigger condition to rising edge.\n");
                result = SA_CTL_SetProperty_i32(dHandle, 0, SA_CTL_PKEY_DEV_INPUT_TRIG_CONDITION, SA_CTL_TRIGGER_CONDITION_RISING);
                exitOnError(result);
                printf("Configure input trigger mode to emergency stop mode.\n");
                result = SA_CTL_SetProperty_i32(dHandle, 0, SA_CTL_PKEY_DEV_INPUT_TRIG_MODE, SA_CTL_DEV_INPUT_TRIG_MODE_EMERGENCY_STOP);
                exitOnError(result);

                printf("Start movement with velocity: 1um/s, distance: 2mm.\n");
                // Set move mode to relative movement
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_RELATIVE);
                exitOnError(result);
                // Set move velocity to 10um/s
                result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 10000000);
                exitOnError(result);
                // Set move acceleration to 10 mm/s2.
                result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 10000000000);
                exitOnError(result);
                result = SA_CTL_Move(dHandle, channel, 2000000000, 0);
                exitOnError(result);
                // Wait for the trigger
                printf("Now waiting for the emergency stop to be triggered...\n");
                waitForEvent();
                // Disable the input trigger
                result = SA_CTL_SetProperty_i32(dHandle, 0, SA_CTL_PKEY_DEV_INPUT_TRIG_MODE, SA_CTL_DEV_INPUT_TRIG_MODE_DISABLED);
                exitOnError(result);
            } else {
                printf("Skipping sequence.\n");
            }
        } else {
            ret = '#';
            printf("*******************************************************\n");
            printf("Second sequence:\n");
            printf("- Position compare output trigger -\n");
            printf("-> press key to [c]ontinue or [s]kip.\n");
            while (!(ret == 'c' || ret == 's')) {
                ret = _getch();
            }
            if (ret == 'c') {
                // Position compare output trigger:
                // We need to set the following properties to configure the position compare output trigger for channel 0:
                // 1. the channel properties specific to the position compare output trigger (start threshold, increment, direction)
                // 2. the channel output trigger mode (behavior of the output trigger function)
                // 4. the general channel output trigger properties, which are not specific to the position compare function
                //    (output trigger polarity, pulse width of the digital output signal)
                // 5. IO module settings (output signal voltage, output driver enabled)

                // Note that the output trigger is dedicated to a specific channel (channel 0 in this example)
                // while the IO module settings are global for the whole IO module (all 3 channels of the module).
                // The output trigger mode should be the last property to set after all previous configuration was set.

                // Afterwards we start a long movement.
                // The channel will generate a 1 us pulse (0.5 us high, 0.5 us low) once the position of channel 0 passed
                // 25um in rising direction. Furthermore every 50 um consecutive pulses are output.

                // Since the direction is configured to "forward", trigger pulses are only generated in this movement direction.
                // Lets say, the positioner passed the starting threshold of 25um -> a pulse is output.
                // Now the threshold is incremented internally by the configured increment of 50um -> the next trigger pulse will be
                // output once the positioner passed the position of 75um, and so on.

                // Set current position to zero
                result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_POSITION, 0);
                exitOnError(result);
                // Set start position to 25um
                result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_CH_POS_COMP_START_THRESHOLD, 25000000);
                exitOnError(result);
                // Set position increment to 50um
                result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_CH_POS_COMP_INCREMENT, 50000000);
                exitOnError(result);
                // Set direction to forward
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_CH_POS_COMP_DIRECTION, SA_CTL_FORWARD_DIRECTION);
                exitOnError(result);
                // disable limits
                result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_CH_POS_COMP_LIMIT_MIN, 0);
                exitOnError(result);
                result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_CH_POS_COMP_LIMIT_MAX, 0);
                exitOnError(result);
                // Set output polarity to active high - ____|^|_________  (low while idle, high when the trigger occurs)
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_CH_OUTPUT_TRIG_POLARITY, SA_CTL_TRIGGER_POLARITY_ACTIVE_HIGH);
                exitOnError(result);
                // Set pulse width to be 0.5 us high and 0.5 us low.
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_CH_OUTPUT_TRIG_PULSE_WIDTH, 1000);
                exitOnError(result);
                printf("Set output trigger mode for channel %d to position compare.\n", channel);
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_CH_OUTPUT_TRIG_MODE, SA_CTL_CH_OUTPUT_TRIG_MODE_POSITION_COMPARE);
                exitOnError(result);
                printf("Configure the I/O module: (3.3V output voltage, driver enabled)\n");
                // Set output driver voltage level to 3.3V.
                result = SA_CTL_SetProperty_i32(dHandle, 0, SA_CTL_PKEY_IO_MODULE_VOLTAGE, SA_CTL_IO_MODULE_VOLTAGE_3V3);
                exitOnError(result);
                // Enable the digital output driver circuit of the I/O module.
                result = SA_CTL_SetProperty_i32(dHandle, 0, SA_CTL_PKEY_IO_MODULE_OPTIONS, SA_CTL_IO_MODULE_OPT_BIT_DIGITAL_OUTPUT_ENABLED);

                printf("Start movement with velocity: 10um/s, distance: 2mm.\n");
                // Set move mode to relative movement
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_RELATIVE);
                exitOnError(result);
                // Set move velocity to 50um/s
                result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 50000000);
                exitOnError(result);
                // Set move acceleration to 10 mm/s2.
                result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 10000000000);
                exitOnError(result);
                result = SA_CTL_Move(dHandle, channel, 2000000000, 0);
                exitOnError(result);
                printf("Starting at 25um, every 50um consecutive pulses are output\non the channel %d fast digital output...\n", channel);
                printf("Press any key to stop the movement.\n");
                _getch();
                // Stop the movement and set output trigger to constant
                result = SA_CTL_Stop(dHandle, channel, 0);
                exitOnError(result);
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_CH_OUTPUT_TRIG_MODE, SA_CTL_CH_OUTPUT_TRIG_MODE_CONSTANT);
                exitOnError(result);
                // Disable the output driver circuit of the I/O module.
                result = SA_CTL_SetProperty_i32(dHandle, 0, SA_CTL_PKEY_IO_MODULE_OPTIONS, 0);
                exitOnError(result);
            } else {
                printf("Skipping sequence.\n");
            }
        }
    }

    printf("Done. Press any key to exit.\n");
    _getch();

    // Before closing the program the connection to the device must be closed by calling "SA_CTL_Close".
    SA_CTL_Close(dHandle);
    printf("MCS2 close.\n");
    printf("*******************************************************\n");
    return 0;
}
