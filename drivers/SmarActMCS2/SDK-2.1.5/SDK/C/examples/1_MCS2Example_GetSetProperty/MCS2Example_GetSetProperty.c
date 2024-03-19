/*
* SmarAct MCS2 programming example: GetProperty/SetProperty
*
* This programming example shows you how to connect to a
* SmarAct MCS2 device and how to use the blocking and non-blocking
* GetProperty and SetProperty functions to read and write
* device properties.
*
* For a full command reference see the MCS2 Programmers Guide.
*
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include "SmarActControl.h"

// Before being able to communicate with a device it must be initialized with a call to "SA_CTL_Open".
// This function connects to the device specified in the locator parameter and returns a handle to the
// device, if the call was successful. The returned device handle must be saved within the application
// and passed as a parameter to the other API functions. Once the connection is established,
// all the other functions may be used to interact with the connected device.
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

    printf("*******************************************************\n");
    printf("*  SmarAct MCS2 Programming Example (Get/Set-Property)*\n");
    printf("*******************************************************\n");

    // Read the version of the library
    // Note: this is the only function that does not require the library to be initialized.
    const char *version = SA_CTL_GetFullVersionString();
    printf("SmarActCTL library version: %s.\n", version);

    // MCS2 devices are identified with locator strings, similar to URLs used to locate web pages.
    // Typical locators are:
    // usb:sn:<serial>       - where <serial> is the device serial which is printed on the housing of the device.
    //                         Example: usb:sn:MCS2-00000412
    // usb:ix:<n>            - where the number <n> selects the nth device in the list of all currently attached
    //                         devices with a USB interface. Example: usb:ix:0.
    // network:sn:<serial>   - where <serial> is the device serial which is printed on the housing of the device.
    //                         Example: network:sn:MCS2-00000412
    // network:192.168.1.200 - where <ip> is an IPv4 address which consists of four integer numbers between 0 and 255
    //                         separated by a dot. Example: network:192.168.1.200
    // It is possible to list all available devices using the "SA_CTL_FindDevices" function.
    // This is shown in the further examples. Here we simply use a fixed locator string.
    char locator[] = { "usb:ix:0" };

    // Note that there is no explicit communication mode (like synchronous or asynchronous)
    // the user must decide upon, as it was in the previous controller systems.
    // The application may decide on a per-call basis which method to use,
    // thus being very flexible depending on the applications context.

    // Additional configuration parameters (unused for now)
    char config[] = {""};

    // All functions return a result code of the type "SA_CTL_Result_t".
    // The return value indicates if the call was successful (SA_CTL_ERROR_NONE) or if an error occurred.
    SA_CTL_Result_t result;

    // Open the MCS2 device
    result = SA_CTL_Open(&dHandle, locator, config);
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to open \"%s\".\n",locator);
    }
    exitOnError(result);
    printf("MCS2 opened \"%s\".\n", locator);

    // Modifying or retrieving property values takes a major role in controlling a device by software.
    // Therefore, the API offers a variety of functions to get and set property values in order to meet all
    // requirements an application might have.

    // Basically there are GetProperty and SetProperty functions for the datatypes "string", "i32" (signed 32bit integer)
    // and "i64" (signed 64bit integer). The "i32" and "i64" versions may be used to process a single value
    // as well as an array of values.
    // Depending on the data type of the specific property, the corresponding variant of the the function must be used.
    // See the MCS2 Programmers Guide for a list of properties and their data types.
    // The passed "idx" parameter specifies the addressed module or channel depending on the specific property.
    // For system properties this parameter is unused and must be set to zero.
    // The "property key" defines the actual property. The predefined keys from the "SmarActControlConstants.h"
    // header file can be used.
    // The following code shows how to read several properties with different datatypes.

    // The easiest method to get a property value is the synchronous (blocking) read
    // consisting of only one simple function call.

    // First we read the device serial number which is a string property using "SA_CTL_GetProperty_s".
    // The "ioStringSize" parameter must contain the size of the passed value buffer and
    // contains the length of the string when the function returns.
    // In case the string size exceeds the provided buffer size an "SA_CTL_ERROR_INVALID_QUERYBUFFER_SIZE" is returned.
    char deviceSN[SA_CTL_STRING_MAX_LENGTH+1];
    size_t ioStringSize = sizeof(deviceSN);
    result = SA_CTL_GetProperty_s(dHandle, 0, SA_CTL_PKEY_DEVICE_SERIAL_NUMBER, deviceSN, &ioStringSize);
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to read the device serial number.\n");
    }
    exitOnError(result);
    printf("MCS2 device serial number: \"%s\".\n", deviceSN);

    // Reading the number of channels of the system using "SA_CTL_GetProperty_i32".
    // Note that the "idx" parameter is unused for this property and thus must be set to zero.
    // Furthermore the "ioArraySize" parameter can also be set to zero since we read a single value and not an array.
    int32_t noOfchannels;
    result = SA_CTL_GetProperty_i32(dHandle, 0, SA_CTL_PKEY_NUMBER_OF_CHANNELS, &noOfchannels, 0);
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to get number of channels.\n");
    }
    exitOnError(result);
    printf("MCS2 number of channels: %d.\n", noOfchannels);

    // Now we read the state for all available channels.
    int32_t state;
    // The passed "idx" parameter (the channel index in this case) is zero-based.
    for (int8_t channel = 0; channel < noOfchannels; channel++) {
        result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
        if (result != SA_CTL_ERROR_NONE) {
            printf("MCS2 failed to get state of channel: %d.\n", channel);
            break;
        }
        // The returned channel state holds a bit field of several state flags.
        // See the MCS2 Programmers Guide for the meaning of all state flags.
        // We pick the "sensorPresent" flag to check if there is a positioner connected
        // which has an integrated sensor.
        // Note that in contrast to previous controller systems the controller supports
        // hotplugging of the sensor module and the actuators.
        if (state & SA_CTL_CH_STATE_BIT_SENSOR_PRESENT) {
            printf("MCS2 channel %d has a sensor.\n", channel);
        } else {
            printf("MCS2 channel %d has no sensor.\n", channel);
        }
    }
    exitOnError(result);

    // For the following steps we need a positioner connected to channel 0.
    int8_t channel = 0;

    // First we want to know if the configured positioner type is a linear or a rotatory type.
    // For this purpose we can read the base unit property.
    int32_t baseUnit;
    result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_BASE_UNIT, &baseUnit, 0);
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to get base unit.\n");
    }
    exitOnError(result);

    // Next we read the current position of channel 0. Position values have the data type int64,
    // thus we need to use "SA_CTL_getProperty_i64".
    // Note that there is no distinction between linear and rotatory positioners regarding the functions which
    // need to be used (getPosition / getAngle) and there is no additional "revolutions" parameter for rotatory positioners
    // as it was in the previous controller systems.
    // Depending on the preceding read base unit, the position is in pico meter [pm] for linear positioners
    // or nano degree [ndeg] for rotatory positioners.
    // Note: it is also possible to read the base resolution of the unit using the property key "SA_CTL_PKEY_POS_BASE_RESOLUTION".
    // To keep things simple this is not shown in this example.
    int64_t position;
    result = SA_CTL_GetProperty_i64(dHandle, channel, SA_CTL_PKEY_POSITION, &position, 0);
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to get position.\n");
    }
    exitOnError(result);
    printf("MCS2 position of channel %d: %" PRId64, channel, position);
    if (baseUnit == SA_CTL_UNIT_METER) printf("pm.\n");
    else printf("ndeg.\n"); // (baseUnit == SA_CTL_UNIT_DEGREE)

    // To show the use of the setProperty function, we set the position to 100 um respectively 100 milli degree.
    // This is the synchronous (blocking) method. The function call blocks until the property value was sent to
    // the controller and the reply was received.
    position = 100000000; // in pm | ndeg
    printf("MCS2 set position of channel %d to %" PRId64, channel, position);
    if (baseUnit == SA_CTL_UNIT_METER) printf("pm.\n");
    else printf("ndeg.\n"); // (baseUnit == SA_CTL_UNIT_DEGREE)
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_POSITION, position);
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to set position.\n");
    }
    exitOnError(result);

    // Now we want to read the the position again (and the channel state in addition).
    // This time we use the asynchronous (non-blocking) method.
    // This method requires two function calls for getting one property value.
    // One for requesting the property value and one for retrieving the answer.
    // The advantage of this method is that the application may request several property values in fast
    // succession and then perform other tasks before blocking on the reception of the results.

    // Received values can later be accessed via the obtained request ID and the corresponding ReadProperty functions.
    SA_CTL_RequestID_t rId[2];

    // The tHandle parameter is used for command groups and output buffering, which is not shown in this example.
    // By passing the transmit handle the request is associated with the command group / output buffer.
    // The transmit handle must be set to zero if command groups / output buffers are not used.

    // Issue requests for the two properties "position" and "channel state".
    result = SA_CTL_RequestReadProperty(dHandle, channel, SA_CTL_PKEY_POSITION, &rId[0], 0);
    exitOnError(result);
    // The function call returns immediately, allowing the application to issue another request or to perform other tasks.
    // We simply request a second property. (the channel state in this case)
    result = SA_CTL_RequestReadProperty(dHandle, channel, SA_CTL_PKEY_CHANNEL_STATE, &rId[1], 0);
    exitOnError(result);

    // ...process other tasks...

    // Receive the results
    // While the request-function is non-blocking the read-functions block until the desired data has arrived.
    // Note that we must use the correct "SA_CTL_ReadProperty_ixx" function depending on the datatype of the requested property.
    // Otherwise an "SA_CTL_ERROR_INVALID_DATA_TYPE" is returned.
    result = SA_CTL_ReadProperty_i64(dHandle, rId[0], &position, 0);
    exitOnError(result);
    result = SA_CTL_ReadProperty_i32(dHandle, rId[1], &state, 0);
    exitOnError(result);

    // Print the results
    printf("MCS2 current position of channel %d: %" PRId64, channel, position);
    if (baseUnit == SA_CTL_UNIT_METER) printf("pm.\n");
    else printf("ndeg.\n"); // (baseUnit == SA_CTL_UNIT_DEGREE)
    if ((state & SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING) == 0) {
        printf("MCS2 channel %d is stopped.\n", channel);
    }

    // For the sake of completeness, finally we use the asynchronous (non-blocking) write function to
    // set the position to -0.1 mm respectively -100 degree.
    position = -100000000;
    printf("MCS2 set position of channel %d to %" PRId64, channel, position);
    if (baseUnit == SA_CTL_UNIT_METER) printf("pm.\n");
    else printf("ndeg.\n"); // (baseUnit == SA_CTL_UNIT_DEGREE)
    result = SA_CTL_RequestWriteProperty_i64(dHandle, channel, SA_CTL_PKEY_POSITION, position, &rId[0], 0);
    // The function call returns immediately, without waiting for the reply from the controller.
    exitOnError(result);

    // ...process other tasks...

    // Wait for the result to arrive.
    result = SA_CTL_WaitForWrite(dHandle, rId[0]);
    exitOnError(result);

    // Alternatively, the "call-and-forget" mechanism for asynchronous (non-blocking) write functions
    // may be used:
    // For property writes the result is only used to report errors. With the call-and-forget mechanism
    // the device does not generate a result for writes and the application can continue processing other
    // tasks immediately. Compared to asynchronous accesses, the application doesn’t need to keep
    // track of open requests and collect the results at some point. This mode should be used with care
    // so that written values are within the valid range.
    // The call-and-forget mechanism is used by passing a null pointer for the request ID pointer to the
    // SA_CTL_RequestWriteProperty_x functions.
    result = SA_CTL_RequestWriteProperty_i64(dHandle, channel, SA_CTL_PKEY_POSITION, position, NULL, 0);
    exitOnError(result);
    // No result must be requested with the SA_CTL_WaitForWrite function in this case.

    // Before closing the program the connection to the device must be closed by calling "SA_CTL_Close".
    SA_CTL_Close(dHandle);
    printf("MCS2 close.\n");

    printf("*******************************************************\n");
    printf("Done. Press return to exit.\n");
    getchar();
    return 0;
}
