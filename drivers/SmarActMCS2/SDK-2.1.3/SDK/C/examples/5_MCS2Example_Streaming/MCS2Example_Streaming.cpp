/*
* SmarAct MCS2 programming example: Streaming
*
* This programming example shows the MCS2 trajectory streaming.
*
* For a full command reference see the MCS2 Programmers Guide.
*
*/

#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <atomic>
#include "SmarActControl.h"

using namespace std;

#define NUMBER_OF_STREAMING_CHANNELS            2
#define MAX_NUMBER_OF_FRAMES                    1024
#define STREAM_FILE_NAME                        "streamPoses.csv"

typedef union int64 {
    int64_t value;              // signed long value
    uint8_t data[8];            // raw data (LSB first)
} INT64_U;

// Each target position is a tuple of a channel index and a position value.
typedef struct  {
    uint8_t channel;
    INT64_U position;
} Tuple_t;

// A trajectory stream is defined as a sequence of support points (frames).
// Each frame is an array of target positions for all channels that participate in the trajectory.
typedef struct {
    Tuple_t frame[NUMBER_OF_STREAMING_CHANNELS];
} Frame_t;

// Create stream buffer
Frame_t streamBuffer[MAX_NUMBER_OF_FRAMES];

// status flags, used for inter-thread communication
atomic_bool streamDone(false);
atomic_bool streamAbort(false);

SA_CTL_DeviceHandle_t dHandle;

void exitOnError(SA_CTL_Result_t result) {
    if (result != SA_CTL_ERROR_NONE) {
        SA_CTL_Close(dHandle);
        // Passing an error code to "SA_CTL_GetResultInfo" returns a human readable string
        // specifying the error.
        cout << "MCS2 " << SA_CTL_GetResultInfo(result) << " (error: 0x" << std::hex << result << ")" << endl;
        cout << "Press return to exit." << endl;
        cin.get();
        exit(1);
    }
}

void waitForEvent(void) {
    SA_CTL_Event_t event;
    while (1) {
        SA_CTL_Result_t result;
        result = SA_CTL_WaitForEvent(dHandle, &event, SA_CTL_INFINITE);
        // we use "SA_CTL_INFINITE", so the function call will return only when canceled by "SA_CTL_Cancel"
        if (result == SA_CTL_ERROR_CANCELED) {
            cout << "MCS2 canceled wait for event" << endl;
            return;
        } else if (result == SA_CTL_ERROR_NONE) {
            // The "type" field specifies the event.
            // The "idx" field holds the channel where the event came from.
            // The "i32" data field gives additional information about the event, e.g. error code.
            // Passing the event to "SA_CTL_GetEventInfo" returns a human readable string
            // specifying the event.
            switch (event.type) {
                case SA_CTL_EVENT_STREAM_FINISHED:
                    cout << "MCS2 " << SA_CTL_GetEventInfo(&event) << endl;
                    if (SA_CTL_EVENT_PARAM_RESULT(event.i32) == SA_CTL_ERROR_NONE) {
                        // All streaming frames were processed, stream finished.
                        streamDone = true;
                    } else if (SA_CTL_EVENT_PARAM_RESULT(event.i32) == SA_CTL_ERROR_ABORTED) {
                        // Stream was canceled by the user.
                        cout << "MCS2 stream aborted by user" << endl;
                        streamDone = true;
                        streamAbort = true;
                    } else {
                        // Stream was canceled by device.
                        // Note: The event parameter now holds the error code as well as the channel index responsible for the failure
                        cout << "MCS2 stream aborted by device" << endl;
                        streamDone = true;
                        streamAbort = true;
                    }
                    break;
                case SA_CTL_EVENT_STREAM_READY:
                case SA_CTL_EVENT_STREAM_TRIGGERED:
                    // These events are mainly useful when the SA_CTL_STREAM_TRIGGER_MODE_EXTERNAL_ONCE trigger mode is used.
                    // A SA_CTL_EVENT_STREAM_READY event is generated to indicate that the stream is ready to be triggered
                    // by the external trigger. In this armed state the device waits for the trigger to occur and then generates a
                    // SA_CTL_EVENT_STREAM_TRIGGERED event.
                    // This example uses the SA_CTL_STREAM_TRIGGER_MODE_DIRECT trigger mode, thus we don't care about this events here.
                    break;
                default:
                    // The code should be prepared to handle unexpected events beside the expected ones.
                    cout << "MCS2 received event: " << SA_CTL_GetEventInfo(&event) << endl;
                    break;
                }
        } else {
            cout << "MCS2 wait for event failed: " << SA_CTL_GetResultInfo(result) << endl;
            return;
        }
    }
    return;
}

int main() {

    SA_CTL_Result_t result;
    cout << "*******************************************************" << endl;
    cout << "*  SmarAct MCS2 Programming Example (Streaming)       *" << endl;
    cout << "*******************************************************" << endl;

    // Read the version of the library
    // Note: this is the only function that does not require the library to be initialized.
    const char *version = SA_CTL_GetFullVersionString();
    cout << "SmarActCTL library version: " << version << endl;

    // Open csv file and fill stream buffer
    // The file format must be:
    // - one frame per line
    // - four numeric values separated by commas
    // <channelA>,<positionA>,<channelB>,<positionB>
    // ...
    // - up to 1024 lines
    // - first line is ignored (title)

    string line;
    ifstream file (STREAM_FILE_NAME);
    if (!file.is_open()) {
        cout << "Failed to open pose file: \"" << STREAM_FILE_NAME << "\"" << endl;
        cin.get();
        exit(1);
    }

    // Consume the head line
    getline(file, line);
    uint32_t noOfFrames = 0;
    uint32_t chA, chB;
    int64_t posA, posB;
    bool valid = true;
    while (getline(file, line)) {
        // ignore blank lines
        if (line.empty()) continue;
        sscanf(line.c_str(), "%d,%lld,%d,%lld\n", &chA, &posA, &chB, &posB);
        // basic valid checks
        if ((chA < 0) || (chB < 0) || (chA > chB)) {
            // channels must be in ascending order
            valid = false;
            break;
        }
        if (noOfFrames >= MAX_NUMBER_OF_FRAMES) break;
        // Write frames to buffer
        streamBuffer[noOfFrames].frame[0].channel = chA;
        streamBuffer[noOfFrames].frame[0].position.value = posA;
        streamBuffer[noOfFrames].frame[1].channel = chB;
        streamBuffer[noOfFrames].frame[1].position.value = posB;
        noOfFrames++;
    }
    file.close();
    if (!valid) {
        cout << "File format invalid." << endl;
        cin.get();
        exit(1);
    } else {
        cout << "Read " << noOfFrames << " stream frames from file \"" << STREAM_FILE_NAME << "\"" << endl;
    }

    // Open the first USB MCS2 found.
    string locator("usb:ix:0");

    result = SA_CTL_Open(&dHandle, locator.c_str(), "");
    if (result != SA_CTL_ERROR_NONE) {
        cout << "MCS2 failed to open \"" << locator.c_str() << "\"." << endl;
    }
    exitOnError(result);
    cout << "MCS2 opened \"" << locator.c_str() << "\"." << endl;

    cout << "*******************************************************" << endl;
    cout << "-> Press return to start position streaming." << endl;
    cin.get();

    // Spawn a thread to receive events from the controller.
    std::thread t(waitForEvent);

    // Set position zero, enable amplifier
    // Sensor power mode: enabled (disable power save, which is not allowed with position streaming)
    for (uint32_t i = 0; i < NUMBER_OF_STREAMING_CHANNELS; i++) {
        result = SA_CTL_SetProperty_i64(dHandle, i, SA_CTL_PKEY_POSITION, 0);
        exitOnError(result);
        result = SA_CTL_SetProperty_i32(dHandle, i, SA_CTL_PKEY_AMPLIFIER_ENABLED, SA_CTL_TRUE);
        exitOnError(result);
        result = SA_CTL_SetProperty_i32(dHandle, i, SA_CTL_PKEY_SENSOR_POWER_MODE, SA_CTL_SENSOR_MODE_ENABLED);
        exitOnError(result);
    }

    // Configure stream (optional)
    // Note: the stream rate must be a whole-number multiplier of the external sync rate.

    // Set external sync rate to 100Hz (only active when using trigger mode SA_CTL_STREAM_TRIGGER_MODE_EXTERNAL_SYNC)
    result = SA_CTL_SetProperty_i32(dHandle, 0, SA_CTL_PKEY_STREAM_EXT_SYNC_RATE, 100);
    exitOnError(result);
    // Set stream base rate to 1kHz
    result = SA_CTL_SetProperty_i32(dHandle, 0, SA_CTL_PKEY_STREAM_BASE_RATE, 1000);
    exitOnError(result);

    // Prepare for streaming, select desired trigger mode
    // (using SA_CTL_STREAM_TRIGGER_MODE_DIRECT starts the stream as soon as enough frames were sent to the device)
    SA_CTL_StreamHandle_t sHandle;
    result = SA_CTL_OpenStream(dHandle, &sHandle, SA_CTL_STREAM_TRIGGER_MODE_DIRECT);
    exitOnError(result);

    // Send all frames in a loop
    // Note: the "SA_CTL_AbortStream" function could be used to abort a running stream programmatically.
    uint32_t frameIdx = 0;
    while (frameIdx < noOfFrames) {
        // The "waitForEvent" thread received an "abort" event.
        if (streamAbort) break;
        // Make byte array from stream data, each frame contains all
        // target positions for all channels that participate in the trajectory.
        // The frame data must have the structure:
        // <chA[one byte]>,<posA[eight bytes],<chB[one byte]>,<posB[eight bytes]>
        // -> 18 bytes in total for this configuration
        uint8_t frameData[NUMBER_OF_STREAMING_CHANNELS*(1+8)];
        uint8_t *pFrame = frameData;
        // channel A
        *pFrame++ = streamBuffer[frameIdx].frame[0].channel;
        for (uint32_t i=0;i<8;i++) *pFrame++ = streamBuffer[frameIdx].frame[0].position.data[i];
        // channel B
        *pFrame++ = streamBuffer[frameIdx].frame[1].channel;
        for (uint32_t i=0; i<8; i++) *pFrame++ = streamBuffer[frameIdx].frame[1].position.data[i];
        // Send frame to the controller
        result = SA_CTL_StreamFrame(dHandle, sHandle, frameData, sizeof(frameData));
        exitOnError(result);
        frameIdx++;
    }


    // All frames sent, close stream
    result = SA_CTL_CloseStream(dHandle, sHandle);
    exitOnError(result);

    // Wait for the "stream done" event.
    while (!streamDone);

    // Cancel waiting for events.
    result = SA_CTL_Cancel(dHandle);
    exitOnError(result);

    // Wait for the "waitForEvent" thread to terminate.
    t.join();
    cout << "Done. Press return to exit." << endl;
    getchar();

    // Before closing the program the connection to the device must be closed by calling "SA_CTL_Close".
    SA_CTL_Close(dHandle);
    cout << "MCS2 close." << endl;
    cout << "*******************************************************" << endl;
    return 0;
}
