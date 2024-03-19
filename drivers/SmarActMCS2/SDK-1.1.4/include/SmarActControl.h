/**********************************************************************
* Copyright (c) 2019 SmarAct GmbH
*
* File name: SmarActControl.h
*
* THIS  SOFTWARE, DOCUMENTS, FILES AND INFORMATION ARE PROVIDED 'AS IS'
* WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
* BUT  NOT  LIMITED  TO,  THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY,
* FITNESS FOR A PURPOSE, OR THE WARRANTY OF NON-INFRINGEMENT.
* THE  ENTIRE  RISK  ARISING OUT OF USE OR PERFORMANCE OF THIS SOFTWARE
* REMAINS WITH YOU.
* IN  NO  EVENT  SHALL  THE  SMARACT  GMBH  BE  LIABLE  FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL, CONSEQUENTIAL OR OTHER DAMAGES ARISING
* OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE.
**********************************************************************/

#ifndef SMARACT_CTL_H
#define SMARACT_CTL_H

#include <stdlib.h>
#include <stdint.h>
#include <SmarActControlConstants.h>

#define SA_CTL_VERSION_MAJOR    1
#define SA_CTL_VERSION_MINOR    3
#define SA_CTL_VERSION_UPDATE   5

#if defined(_WIN32)
#  define SA_CTL_PLATFORM_WINDOWS
#elif defined(__linux__)
#  define SA_CTL_PLATFORM_LINUX
#else
#  error "unsupported platform"
#endif


#if defined(SA_CTL_PLATFORM_WINDOWS)
#  if !defined(_SA_CTL_DIRECTLINK)
#    ifdef SA_CTL_EXPORTS
#      define SA_CTL_API __declspec(dllexport)
#    else
#      define SA_CTL_API __declspec(dllimport)
#    endif
#  else
#    define SA_CTL_API
#  endif
#  define SA_CTL_CC __cdecl
#else
#  define SA_CTL_API __attribute__ ((visibility ("default")))
#  define SA_CTL_CC
#endif




#ifdef __cplusplus
extern "C" {
#endif

/************
 * TYPEDEFS *
 ***********/

typedef uint32_t SA_CTL_DeviceHandle_t;
typedef uint32_t SA_CTL_TransmitHandle_t;
typedef uint32_t SA_CTL_StreamHandle_t;

typedef uint8_t SA_CTL_RequestID_t;
typedef uint32_t SA_CTL_PropertyKey_t;
typedef uint32_t SA_CTL_Result_t;

typedef struct {
    uint32_t idx;
    uint32_t type;
    union {
        int32_t i32;
        int64_t i64;
        uint8_t unused[24];
    };
} SA_CTL_Event_t;

/**************************
* MISCELLANEOUS FUNCTIONS *
**************************/

/* returns the version of the library as a human readable string */
SA_CTL_API
const char* SA_CTL_CC SA_CTL_GetFullVersionString();

/* returns a human readable string for the given result code */
SA_CTL_API
const char* SA_CTL_CC SA_CTL_GetResultInfo(SA_CTL_Result_t result);

/* returns a human readable info string for the given event */
SA_CTL_API
const char* SA_CTL_CC SA_CTL_GetEventInfo(const SA_CTL_Event_t *event);

/***************************
* INITIALIZATION FUNCTIONS *
***************************/

/* opens a connection to a device specified by a locator string */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_Open(SA_CTL_DeviceHandle_t *dHandle, const char *locator, const char *config);

/* closes a previously established connection to a device */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_Close(SA_CTL_DeviceHandle_t dHandle);

/* aborts waiting functions like SA_CTL_WaitForEvent
   - if no thread is currently waiting, the next call to SA_CTL_WaitForEvent will be canceled
   - the unblocked function will return with an SA_CTL_ERROR_CANCELED error */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_Cancel(SA_CTL_DeviceHandle_t dHandle);

/* returns a list of locator strings of available devices
   - the locator strings are separated by a newline character */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_FindDevices(const char *options, char *deviceList, size_t *deviceListLen);


/***********************************
* BLOCKING READ/WRITE ACCESS - i32 *
***********************************/

/* directly returns the value (array) of a 32-bit integer property
   - ioArraySize is a pointer to a size value that must contain the size of the value buffer (in number of elements, not number of bytes) when the function is called.
     On function return it contains the number of values written to the buffer. A null pointer is allowed which implicitly indicates an array size of 1. */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_GetProperty_i32(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int32_t *value, size_t *ioArraySize);

/* directly sets the value of a 32-bit integer property */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_SetProperty_i32(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int32_t value);

/* directly sets the value of a 32-bit integer array property */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_SetPropertyArray_i32(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const int32_t *values, size_t arraySize);

/***********************************
* BLOCKING READ/WRITE ACCESS - i64 *
***********************************/

/* directly returns the value (array) of a 64-bit integer property
   - ioArraySize is a pointer to a size value that must contain the size of the value buffer (in number of elements, not number of bytes) when the function is called.
     On function return it contains the number of values written to the buffer. A null pointer is allowed which implicitly indicates an array size of 1. */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_GetProperty_i64(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int64_t *value, size_t *ioArraySize);

/* directly sets the value of a 64-bit integer property */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_SetProperty_i64(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int64_t value);

/* directly sets the value of a 64-bit integer array property */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_SetPropertyArray_i64(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const int64_t *values, size_t arraySize);

/**************************************
* BLOCKING READ/WRITE ACCESS - String *
**************************************/

/* directly returns the value of a string (array) property
   - ioArraySize is a pointer to a size value that must contain the size of the value buffer (in bytes) when the function is called.
     On function return it contains the number of characters written to the buffer.
   - the null termination of a string implicitly serves as a separator in case multiple strings are returned */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_GetProperty_s(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, char *value, size_t *ioArraySize);

/* directly sets the value of a string property */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_SetProperty_s(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const char *value);

/****************************/
/* NON-BLOCKING READ ACCESS */
/****************************/

/* requests the value of a property (non-blocking)
   - received values can be accessed later via the obtained request ID and the corresponding SA_CTL_ReadProperty_x functions */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestReadProperty(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle);

/* reads a 32-bit integer property value (array) that has previously been requested using SA_CTL_RequestReadProperty
   - ioArraySize is a pointer to a size value that must contain the size of the value buffer (in number of elements, not number of bytes) when the function is called.
     On function return it contains the number of values written to the buffer. A null pointer is allowed which implicitly indicates an array size of 1.
   - while the request-function is non-blocking the read-functions block until the desired data has arrived */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_ReadProperty_i32(SA_CTL_DeviceHandle_t dHandle, SA_CTL_RequestID_t rID, int32_t *value, size_t *ioArraySize);

/* reads a 64-bit integer property value (array) that has previously been requested using SA_CTL_RequestReadProperty
   - ioArraySize is a pointer to a size value that must contain the size of the value buffer (in number of elements, not number of bytes) when the function is called.
     On function return it contains the number of values written to the buffer. A null pointer is allowed which implicitly indicates an array size of 1.
   - while the request-function is non-blocking the read-functions block until the desired data has arrived */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_ReadProperty_i64(SA_CTL_DeviceHandle_t dHandle, SA_CTL_RequestID_t rID, int64_t *value, size_t *ioArraySize);

/* reads a string property value (array) that has previously been requested using SA_CTL_RequestReadProperty
   - ioArraySize is a pointer to a size value that must contain the size of the value buffer (in bytes) when the function is called.
     On function return it contains the number of characters written to the buffer.
   - the null termination of a string implicitly serves as a separator in case multiple strings are returned
   - while the request-function is non-blocking the read-functions block until the desired data has arrived */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_ReadProperty_s(SA_CTL_DeviceHandle_t dHandle, SA_CTL_RequestID_t rID, char *value, size_t *ioArraySize);

/*****************************/
/* NON-BLOCKING WRITE ACCESS */
/*****************************/

/* requests to write the value of a 32-bit integer property (non-blocking)
   - the result (whether the write was successful or not) can be accessed later by passing the obtained request ID to the SA_CTL_WaitForWrite function */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestWriteProperty_i32(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int32_t value, SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle);

/* requests to write the value of a 64-bit integer property (non-blocking)
   - the result (whether the write was successful or not) can be accessed later by passing the obtained request ID to the SA_CTL_WaitForWrite function */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestWriteProperty_i64(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int64_t value, SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle);

/* requests to write the value of a string property (non-blocking)
   - the result (whether the write was successful or not) can be accessed later by passing the obtained request ID to the SA_CTL_WaitForWrite function */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestWriteProperty_s(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const char *value, SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle);

/* requests to write the value of a 32-bit integer array property (non-blocking)
   - the result (whether the write was successful or not) can be accessed later by passing the obtained request ID to the SA_CTL_WaitForWrite function */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestWritePropertyArray_i32(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const int32_t *values, size_t arraySize, SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle);

/* requests to write the value of a 64-bit integer array property (non-blocking)
   - the result (whether the write was successful or not) can be accessed later by passing the obtained request ID to the SA_CTL_WaitForWrite function */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestWritePropertyArray_i64(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const int64_t *values, size_t arraySize, SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle);

/* returns the result of a property write access that has previously been requested using the data type specific SA_CTL_RequestWriteProperty_x function
   - while the request-function is non-blocking the SA_CTL_WaitForWrite function blocks until the desired result has arrived */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_WaitForWrite(SA_CTL_DeviceHandle_t dHandle, SA_CTL_RequestID_t rID);

/* cancels a non-blocking read or write access
   - note that without output buffering the request has already been sent. In this case only the answer/result will be discarded but property writes will still be executed. */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_CancelRequest(SA_CTL_DeviceHandle_t dHandle, SA_CTL_RequestID_t rID);

/**************************/
/* OUTPUT BUFFER HANDLING */
/**************************/

/* opens an output buffer for delayed transmission of several commands */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_CreateOutputBuffer(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t *tHandle);

/* flushes an output buffer and triggers the transmission to the device */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_FlushOutputBuffer(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t tHandle);

/* cancels an output buffer and discards all buffered commands */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_CancelOutputBuffer(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t tHandle);

/**************************/
/* COMMAND GROUP HANDLING */
/**************************/

/* opens a command group that can be used to combine multiple asynchronous commands into an atomic group */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_OpenCommandGroup(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t *tHandle, uint32_t triggerMode);

/* closes and eventually executes the assembled command group depending on the configured trigger mode */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_CloseCommandGroup(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t tHandle);

/* cancels a command group and discards buffered commands */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_CancelCommandGroup(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t tHandle);

/***************************/
/* WAIT AND RECEIVE EVENTS */
/***************************/

/* listens to events from the device */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_WaitForEvent(SA_CTL_DeviceHandle_t dHandle, SA_CTL_Event_t *event, uint32_t timeout);

/**********************/
/* MOVEMENT FUNCTIONS */
/**********************/

/* starts a calibration routine for a given channel
   - the calibration options property should be configured to define the behavior of the calibration sequence before calling this function */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_Calibrate(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_TransmitHandle_t tHandle);

/* starts a referencing routine for a given channel
   - the referencing options property (as well as the move velocity, acceleration, etc.) should be configured to define the behavior of the
     referencing sequence before calling this function */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_Reference(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_TransmitHandle_t tHandle);

/* instructs a positioner to move according to the current move configuration
   - the move mode as well as corresponding parameters (e.g. frequency, move velocity, holdTime, etc.) have to be configured beforehand */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_Move(SA_CTL_DeviceHandle_t dHandle, int8_t idx, int64_t moveValue, SA_CTL_TransmitHandle_t tHandle);

/* stops any ongoing movement of the given channel */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_Stop(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_TransmitHandle_t tHandle);

/***********************/
/* STREAMING FUNCTIONS */
/***********************/

/* opens a trajectory stream to the device */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_OpenStream(SA_CTL_DeviceHandle_t dHandle, SA_CTL_StreamHandle_t *sHandle, uint32_t triggerMode);

/* supplies the device with stream data by sending one frame per function call
   - a frame contains the data for one interpolation point which must be assembled by concatenating elements of channel index (1 byte) and position (8 byte)
   - this function may block if the flow control needs to throttle the data rate. The function returns as soon as the frame was transmitted to the controller.
   - the desired streamrate as well as the external syncrate have to be configured beforehand */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_StreamFrame(SA_CTL_DeviceHandle_t dHandle, SA_CTL_StreamHandle_t sHandle, const uint8_t *frameData, uint32_t frameSize);

/* closes a trajectory stream */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_CloseStream(SA_CTL_DeviceHandle_t dHandle, SA_CTL_StreamHandle_t sHandle);

/* aborts a trajectory stream */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_AbortStream(SA_CTL_DeviceHandle_t dHandle, SA_CTL_StreamHandle_t sHandle);

#ifdef __cplusplus
}
#endif

#endif // SMARACT_CTL_H
