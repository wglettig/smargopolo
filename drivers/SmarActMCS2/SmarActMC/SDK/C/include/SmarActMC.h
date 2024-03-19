/** ********************************************************************
*
* @mainpage SmarActMC API
*
* This is the C API for the SmarAct Motion Control software.
* See subsections for documentation of functions and constants.
*
* 
* @section Copyright
*
* Copyright (c) 2019-2020 SmarAct GmbH
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
* @file
********************************************************************* */

#ifndef SMARACT_MOTION_CONTROL_H
#define SMARACT_MOTION_CONTROL_H

#include <stdlib.h>
#include <stdint.h>





#if defined(_WIN32)
#  define SA_MC_PLATFORM_WINDOWS
#elif defined(__linux__)
#  define SA_MC_PLATFORM_LINUX
#else
#  error "unsupported platform"
#endif



#ifdef SA_MC_PLATFORM_WINDOWS
#  if !defined(_SA_MC_DIRECTLINK)
#    ifdef SA_MC_EXPORTS
#      define SA_MC_API __declspec(dllexport)
#    else
#      define SA_MC_API __declspec(dllimport)
#    endif
#  else
#    define SA_MC_API
#  endif
#  define SA_MC_CC __cdecl
#else
#  define SA_MC_API __attribute__ ((visibility ("default")))
#  define SA_MC_CC
#endif



#ifdef __cplusplus
extern "C" {
#endif

/*****************************
   API Version
 *****************************/

/** @constants ApiVersion */
#define SA_MC_VERSION_MAJOR                             0
#define SA_MC_VERSION_MINOR                             11
#define SA_MC_VERSION_UPDATE                            4
/** @endconstants ApiVersion */



/** **************************************************************************
   @defgroup ErrorCodes Error Codes
   @{
 ************************************************************************** */

/** @constants ErrorCode */

#define SA_MC_ERROR_NONE                                0x0000

#define SA_MC_ERROR_OTHER                               0x0001

#define SA_MC_ERROR_INVALID_PARAMETER                   0x0002
#define SA_MC_ERROR_INVALID_LOCATOR                     0x0003
#define SA_MC_ERROR_INVALID_HANDLE                      0x0005
#define SA_MC_ERROR_NOT_SUPPORTED                       0x0006
#define SA_MC_ERROR_SOFTWARE_RESOURCE_LIMIT             0x0007
#define SA_MC_ERROR_QUERYBUFFER_SIZE                    0x0008
#define SA_MC_ERROR_WRONG_DATA_TYPE                     0x0009
#define SA_MC_ERROR_NO_ACCESS                           0x000a


#define SA_MC_ERROR_INVALID_PROPERTY                    0x0020


#define SA_MC_ERROR_CANCELED                            0x0100
#define SA_MC_ERROR_TIMEOUT                             0x0101


#define SA_MC_ERROR_POSE_UNREACHABLE                    0x0200
#define SA_MC_ERROR_NOT_REFERENCED                      0x0201
#define SA_MC_ERROR_BUSY                                0x0203


#define SA_MC_ERROR_ENDSTOP_REACHED                     0x0300
#define SA_MC_ERROR_FOLLOWING_ERROR_LIMIT_REACHED       0x0301


#define SA_MC_ERROR_REFERENCING_FAILED                  0x0320

#define SA_MC_ERROR_DRIVER_FAILED                       0x0500
#define SA_MC_ERROR_CONNECT_FAILED                      0x0501
#define SA_MC_ERROR_NOT_CONNECTED                       0x0502
#define SA_MC_ERROR_CONTROLLER_CONFIGURATION            0x0503
#define	SA_MC_ERROR_COMMUNICATION_FAILED                0x0504

/** @endconstants ErrorCode */


/** @} */



/** **************************************************************************
   @defgroup Properties

   Properties are used to set read and write configuration parameters.

   @{
 ************************************************************************** */

/** @constants Property */


#define SA_MC_PKEY_MODEL_CODE                           0x0a02
#define SA_MC_PKEY_MODEL_NAME                           0x0a03



/* Axis and Controller Properties */

#define SA_MC_PKEY_IS_REFERENCED                        0x2a01

#define SA_MC_PKEY_HOLD_TIME                            0x2000
#define SA_MC_PKEY_MAX_SPEED_LINEAR_AXES                0x2010
#define SA_MC_PKEY_MAX_SPEED_ROTARY_AXES                0x2011



/* Piezo Actuation Parameters */

#define SA_MC_PKEY_PIEZO_MAX_CLF_LINEAR_AXES            0x2020
#define SA_MC_PKEY_PIEZO_MAX_CLF_ROTARY_AXES            0x2021

/** @endconstants Property */

/** @} */


/** **************************************************************************
   @defgroup PropertyValues Property Values
   @{
 ************************************************************************** */



/** @} */


/** **************************************************************************
   @defgroup CommonConstants Common Constants
   @{
 ************************************************************************** */

/** @constants Global */

#define SA_MC_FALSE                                     0
#define SA_MC_TRUE                                      1

/** A constant indicating infinity */
#define SA_MC_INFINITE                                  (-1)


/** A handle that refers to no object. */
#define SA_MC_NO_HANDLE                                 0

/** @endconstants Global */


/** @} */




/** **************************************************************************
   @defgroup EventType
   @{
 ************************************************************************** */

/** @constants EventType */


/** This event occurs when a movement has finished or failed. 
    
    The @p i32 parameter of SA_MC_Event contains the status code.
    If it is SA_MC_ERROR_NONE, the movement was successful, otherwise not.
*/
#define SA_MC_EVENT_MOVEMENT_FINISHED                   0x0001

/** @endconstants EventType */


/** @} */




/** **************************************************************************
   @defgroup Types
   @{
 ************************************************************************** */

/** Function return value type. */
typedef int32_t SA_MC_Result;


/** Handle of a successfully opened device. */
typedef uint32_t SA_MC_Handle;


/** Property key type. */
typedef uint32_t SA_MC_PropertyKey;


/** Pose data. 

    A pose decribes the translation and orientation of a body in space.
    (x,y,z) is the translation, and (rx,ry,rz) the rotation.
*/
typedef struct SA_MC_Pose {
    double x;       /**< Translation parallel to the X axis [m] */
    double y;       /**< Translation parallel to the Y axis [m] */
    double z;       /**< Translation parallel to the Z axis [m] */
    double rx;      /**< Rotation around the X axis in [deg]    */
    double ry;      /**< Rotation around the Y axis in [deg]    */
    double rz;      /**< Rotation around the Z axis in [deg]    */
}SA_MC_Pose;


/** A point in 3D space. */
typedef struct SA_MC_Vec3 {
    double x, y, z;
}SA_MC_Vec3;


/** Event description structure.

    Events are received with SA_MC_WaitForEvent().
*/
typedef struct SA_MC_Event {
    uint32_t type;              /**< The event type */
    uint8_t unused[28];         /**< Reserved, don't use */
    union {
        int32_t i32;            /**< Event parameter, int32_t type */
        int64_t i64;            /**< Event parameter, int64_t type */
        uint8_t reserved[32];   /**< Reserved, don't use */
    };
}SA_MC_Event;

/** @} */


/** **************************************************************************
   @defgroup Functions
   @{
 ************************************************************************** */


/** Returns a string representation for a result code.

   Function returns a C string pointer in the @p info parameter that 
   describes the @p result code.

   @returns @ref SA_MC_ERROR_INVALID_PARAMETER if @p resultCode is not a known result code.
   @param[in] resultCode The result code.
   @param[out] info A pointer to a C string.
*/
SA_MC_API 
SA_MC_Result SA_MC_CC SA_MC_GetResultInfo(SA_MC_Result resultCode, const char **info);



/** Searches for controllers and returns a list of controller locators.

   @param[in] options Reserved for now.
   @param[out] outBuffer The buffer for writing the result string into.
            The locators of the found controllers are separated by
            by a newline character.
   @param[in,out] bufferSize Pass the size of the provided buffer in here,
            on return, it contains the number of characters returned
            in outBuffer, or the required size, if the supplied buffer
            was too small.

   @size outBuffer ~bufferSize
   @default bufferSize 256
   @default options ""
*/
SA_MC_API 
SA_MC_Result SA_MC_CC SA_MC_FindControllers(const char *options,
    char *outBuffer, size_t *bufferSize);


/** Opens a device and returns a handle in outHandle.

   @param[out] outHandle The generated handle is returned in *outHandle.

   @param[in] options A list of options. each option must be defined on a new
        line. the format is 
                "[option value]\n[option value]\n..."   
        possible options are:\n
            model   The device model code\n
            locator The locator of the axis controller. E.g.
                        "network:192.168.1.200:5000" for network controllers or
                        "usb:id:98765432" for controllers with USB interface.
                        A network locator contains the IP address and the
                        port of the controller. The USB locator format contains
                        the first part of the MCS controller serial number.

   Example:
   @code
   SA_MC_Handle h;
   SA_MC_Result res = SA_MC_Open(&h,"locator usb:id:98765432\nmodel 21004")
   if(res != SA_MC_ERROR_NONE) {
      //...
   }
   @endcode
*/
SA_MC_API 
SA_MC_Result SA_MC_CC SA_MC_Open(SA_MC_Handle *outHandle, const char *options);


/** Closes device.

   @param[in] handle The device handle.
*/
SA_MC_API 
SA_MC_Result SA_MC_CC SA_MC_Close(SA_MC_Handle handle);


/** References all axes.

   Asynchronous operation, returns immediately.
   To wait for completion of operation, call SA_MC_WaitForEvent()

   @param[in] handle The device handle.
*/
SA_MC_API 
SA_MC_Result SA_MC_CC SA_MC_Reference(SA_MC_Handle handle);


/** Moves to specified pose.

   Asynchronous operation, returns immediately.
   To wait for completion of operation, call SA_MC_WaitForEvent()
   
   @param[in] handle The device handle.
   @param[in] pose The target pose.
*/
SA_MC_API 
SA_MC_Result SA_MC_CC SA_MC_Move(SA_MC_Handle handle, const SA_MC_Pose *pose);


/** Waits for and event.

   A call to the function blocks the current thread until an event
   has occurred or waiting has timed out. 

   Waiting can be canceled from a different thread with SA_MC_Stop().

   @returns SA_MC_ERROR_NONE if an event could be received within the @p timeout.
        (Errors of movements are encoded in the i32 parameter of the 
        @ref SA_MC_EVENT_MOVEMENT_FINISHED event).
   @param[in] handle The device handle.
   @param[out] outEvent The event data returned by the function.
   @param[in] timeout Wait returns with a SA_MC_ERROR_TIMEOUT if the operations
        doesn't finish within @p timeout milliseconds. If @p timeout is 
        @ref SA_MC_INFINITE (-1), the function waits indefinitely.
*/
SA_MC_API 
SA_MC_Result SA_MC_CC SA_MC_WaitForEvent(SA_MC_Handle handle, SA_MC_Event *outEvent, int32_t timeout);


/** Stops a movement and terminates active holding of the target positions.

   All movement operations like referencing are stopped.
   If SA_MC_WaitForEvent() has been called it will return with
   @ref SA_MC_ERROR_CANCELED.

   @param[in] handle The device handle.
*/
SA_MC_API 
SA_MC_Result SA_MC_CC SA_MC_Stop(SA_MC_Handle handle);


/** Cancels calls of WaitForEvent..

   If SA_MC_WaitForEvent() has been called it will return with
   @ref SA_MC_ERROR_CANCELED.

   @param[in] handle The device handle.
*/
SA_MC_API 
SA_MC_Result SA_MC_CC SA_MC_Cancel(SA_MC_Handle handle);


/** Returns the current pose. Device must be referenced.

   @param[in] handle The device handle.
   @param[out] pose The returned pose.
*/
SA_MC_API 
SA_MC_Result SA_MC_CC SA_MC_GetPose(SA_MC_Handle handle,SA_MC_Pose *pose);


/** Sets a given pose as the pivot point.

   @param[in] handle The device handle.
   @param[in] pivot The pivot point.
*/
SA_MC_API
SA_MC_Result SA_MC_CC SA_MC_SetPivot(SA_MC_Handle handle, const SA_MC_Vec3 *pivot);


/** Returns the current pivot point.

   @param[in] handle The device handle.
   @param[out] pivot The returned current pivot point.
*/
SA_MC_API
SA_MC_Result SA_MC_CC SA_MC_GetPivot(SA_MC_Handle handle, SA_MC_Vec3 *pivot);


/** Sets the i32 property value.

   @returns @ref SA_MC_ERROR_INVALID_PROPERTY if @p pkey is an unknown property or
        it is not accessible.
   @param[in] handle The device handle.
   @param[in] pkey The property key.
   @param[in] value The property value.
*/
SA_MC_API
SA_MC_Result SA_MC_CC SA_MC_SetProperty_i32(SA_MC_Handle handle, SA_MC_PropertyKey pkey, int32_t value);


/** Sets the f64 (double) property value.

   @returns @ref SA_MC_ERROR_INVALID_PROPERTY if @p pkey is an unknown property or
        it is not accessible.
   @param[in] handle The handle.
   @param[in] pkey The property key.
   @param[in] value The property value.
*/
SA_MC_API
SA_MC_Result SA_MC_CC SA_MC_SetProperty_f64(SA_MC_Handle handle, SA_MC_PropertyKey pkey, double value);


/** Returns the i32 property value.

   @returns @ref SA_MC_ERROR_INVALID_PROPERTY if @p pkey is an unknown property or
        it is not accessible.
   @param[in] handle The handle.
   @param[in] pkey The property key.
   @param[out] value The property value.
*/
SA_MC_API
SA_MC_Result SA_MC_CC SA_MC_GetProperty_i32(SA_MC_Handle handle, SA_MC_PropertyKey pkey, int32_t *value);


/** Returns the f64 (double) property value.

   @returns @ref SA_MC_ERROR_INVALID_PROPERTY if @p pkey is an unknown property or
        it is not accessible.
   @param[in] handle The handle.
   @param[in] pkey The property key.
   @param[out] value The property value.
*/
SA_MC_API
SA_MC_Result SA_MC_CC SA_MC_GetProperty_f64(SA_MC_Handle handle, SA_MC_PropertyKey pkey, double *value);


/** Returns the string property value.

   @returns @ref SA_MC_ERROR_INVALID_PROPERTY if @p pkey is an unknown property or
        it is not accessible.
   @param[in] handle The handle.
   @param[in] pkey The property key.
   @param[out] outBuffer The buffer to write the string to.
   @param[in,out] bufferSize In: the size of the buffer. 
        Out: the written number of characters +1 (for the string termination 0-byte) 
        if successful or the required buffer size, if not.

   @size outBuffer ~bufferSize
   @default bufferSize 256
*/
SA_MC_API
SA_MC_Result SA_MC_CC SA_MC_GetProperty_s(SA_MC_Handle handle, SA_MC_PropertyKey pkey, char *outBuffer, size_t *bufferSize);


/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SMARACTMC_H */