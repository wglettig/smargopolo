#!/usr/bin/python3
import sys
import smaract.ctl as ctl

# SmarAct MCS2 programming example: Events
#
# This programming example shows you how to
# receive events from an MCS2 device.
# For a full command reference see the MCS2 Programmers Guide.

def assert_lib_compatibility():
    """
    Checks that the major version numbers of the Python API and the
    loaded shared library are the same to avoid errors due to 
    incompatibilities.
    Raises a RuntimeError if the major version numbers are different.
    """
    vapi = ctl.api_version
    vlib = [int(i) for i in ctl.GetFullVersionString().split('.')]
    if vapi[0] != vlib[0]:
        raise RuntimeError("Incompatible SmarActCTL python api and library version.")

def waitForEvent():
    """ Wait for events generated by the connected device """
    # The wait for event function blocks until an event was received or the timeout elapsed.
    # In case of timeout, a "ctl.Error" exception is raised containing the "TIMEOUT" error.
    # If the "timeout" parameter is set to "ctl.INFINITE" the call blocks until an event is received.
    # This can be useful in case the WaitForEvent function runs in a separate thread.
    # For simplicity, this is not shown here thus we set a timeout of 3 seconds.
    timeout = 3000 # in ms
    try:
        event = ctl.WaitForEvent(d_handle, timeout)
        # The "type" field specifies the event.
        # The "idx" field holds the device index for this event, it will always be "0", thus might be ignored here.
        # The "i32" data field gives additional information about the event.
        if event.type == ctl.EventType.MOVEMENT_FINISHED:
            if (event.i32 == ctl.ErrorCode.NONE):
                # Movement finished.
                print("MCS2 movement finished, channel: ", event.idx)
            else:
                # The movement failed for some reason. E.g. an endstop was detected.
                print("MCS2 movement finished, channel: {}, error: 0x{:04X} ({}) ".format(event.idx, event.i32, ctl.GetResultInfo(event.i32)))
        else:
            # The code should be prepared to handle unexpected events beside the expected ones.
            print("MCS2 received event: {}".format(ctl.GetEventInfo(event)))

    except ctl.Error as e:
        if e.code == ctl.ErrorCode.TIMEOUT:
            print("MCS2 wait for event timed out after {} ms".format(timeout))
        else:
            print("MCS2 {}".format(ctl.GetResultInfo(e.code)))
        return

print("*******************************************************")
print("*  SmarAct MCS2 Programming Example (Events)          *")
print("*******************************************************")

# Read the version of the library
# Note: this is the only function that does not require the library to be initialized.
version = ctl.GetFullVersionString()
print("SmarActCTL library version: '{}'.".format(version))
assert_lib_compatibility()

# Find available MCS2 devices
try:
    buffer = ctl.FindDevices()
    if len(buffer) == 0:
        print("MCS2 no devices found.")
        sys.exit(1)
    locators = buffer.split("\n")
    for locator in locators:
        print("MCS2 available devices: {}".format(locator))
except:
    print("MCS2 failed to find devices. Exit.")
    input()
    sys.exit(1)

d_handle = None
try:
    # Open the first MCS2 device from the list
    d_handle = ctl.Open(locators[0])
    print("MCS2 opened {}.".format(locators[0]))

    # Events are asynchronously generated notifications from the controller.
    # They can be used to receive state information from the controller
    # without continuously polling the state.
    # See the MCS2 Programmers Guide for a list of all available events.

    # The following code commands a closed-loop relative movement and uses the
    # "WaitForEvent" function to receive events from the controller.

    channel = 0
    # Set move mode to closed-loop relative movement.
    ctl.SetProperty_i32(d_handle, channel, ctl.Property.MOVE_MODE, ctl.MoveMode.CL_RELATIVE)
    # Set move velocity [in pm/s].
    ctl.SetProperty_i64(d_handle, channel, ctl.Property.MOVE_VELOCITY, 100000000)
    # Set move acceleration [in pm/s2].
    ctl.SetProperty_i64(d_handle, channel, ctl.Property.MOVE_ACCELERATION, 10000000000)
    # Enable the amplifier.
    ctl.SetProperty_i32(d_handle, channel, ctl.Property.AMPLIFIER_ENABLED, ctl.TRUE)

    print("*******************************************************")
    print("-> Press return to start movement.")
    input()
    # Command a closed-loop movement.
    ctl.Move(d_handle, channel, 100000000)
    # Wait for the "MOVEMENT_FINISHED" event.
    waitForEvent()

    # Again commanding a (time consuming) movement. Intentionally provoking a timeout.
    print("-> Press return to start movement.")
    input()
    ctl.Move(d_handle, channel, 500000000)
    # Will return with a timeout.
    waitForEvent()
    # Stop positioner.
    ctl.Stop(d_handle, channel, 0)
    print("Stop channel {}.".format(channel))

except ctl.Error as e:
    # Passing an error code to "GetResultInfo" returns a human readable string
    # specifying the error.
    print("MCS2 {}: {}, error: {} (0x{:04X}) in line: {}."
          .format(e.func, ctl.GetResultInfo(e.code), ctl.Error(e.code).name, e.code, (sys.exc_info()[-1].tb_lineno)))

except Exception as ex:
    print("Unexpected error: {}, {} in line: {}".format(ex, type(ex), (sys.exc_info()[-1].tb_lineno)))
    raise

finally:
    # Before closing the program the connection to the device must be closed by calling "Close".
    if d_handle != None:
        ctl.Close(d_handle)
    print("MCS2 close.")
    print("*******************************************************")
    print("Done. Press return to exit.")
    input()