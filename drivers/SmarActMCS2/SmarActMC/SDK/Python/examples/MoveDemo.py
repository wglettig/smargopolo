# Copyright (c) 2020 SmarAct GmbH

# Programming example for the SmarActMC Python API.

# THIS  SOFTWARE, DOCUMENTS, FILES AND INFORMATION ARE PROVIDED 'AS IS'
# WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
# BUT  NOT  LIMITED  TO,  THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY,
# FITNESS FOR A PURPOSE, OR THE WARRANTY OF NON-INFRINGEMENT.
# THE  ENTIRE  RISK  ARISING OUT OF USE OR PERFORMANCE OF THIS SOFTWARE
# REMAINS WITH YOU.
# IN  NO  EVENT  SHALL  THE  SMARACT  GMBH  BE  LIABLE  FOR ANY DIRECT,
# INDIRECT, SPECIAL, INCIDENTAL, CONSEQUENTIAL OR OTHER DAMAGES ARISING
# OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE.
#
# WHEN RUNNING, THIS PROGRAMMING EXAMPLE WILL MOVE THE POSITIONER. 
# ENSURE THAT THIS CAN'T CAUSE COLLISIONS. THE POSITIONER MUST HAVE
# SUFFICIENT SPACE TO MOVE OVER ITS FULL RANGE.
#
# BEFORE EXECUTING THE PROGRAM, CHECK THAT THE model AND locator
# PARAMETERS MATCH YOUR POSITIONER MODEL CODE AND CONTROLLER LOCATOR.


import time

import smaract.mc as mc
from smaract.mc import Pose as Pose


modelCode = 0               # set this to software model code of your device
locator = "usb:sn:00000000" # set this to the locator string of your controller
force_referencing = False   # set to True to reference even if device is already referenced



def print_info():
    vapi = mc.api_version
    print("api version = %s.%s.%s" % (vapi[0], vapi[1], vapi[2]))
    # vlib = mc.GetDLLVersion()
    # print("lib version = %s.%s.%s" % (vlib[0], vlib[1], vlib[2]))

def pose_to_str(pose):
    return "%s,%s,%s,%s,%s,%s" %  (
        pose.x, pose.y, pose.z,
        pose.rx, pose.ry, pose.rz)


class Delay:
    delay = 0
    def __init__(self, d):
        self.delay = d

class Velocity:
    velLin = 0
    velRot = 0
    def __init__(self, vlin, vrot):
        self.velLin = vlin
        self.velRot = vrot


def exec_commandlist(h,clist):
    """
    executes commands in list clist.
    commands can be objects of type Delay, Velocity and Pose
    """
    for sp in clist:
        if type(sp) is Delay:
            print("waiting %s ms" % sp.delay)
            time.sleep(sp.delay * 1e-3)

        elif type(sp) is Velocity:
            print("max velocity: linear = %s, rotary = %s" % (sp.velLin,sp.velRot))
            mc.SetProperty_f64(h, mc.Property.MAX_SPEED_LINEAR_AXES, sp.velLin)    
            mc.SetProperty_f64(h, mc.Property.MAX_SPEED_ROTARY_AXES, sp.velRot) 

        elif type(sp) is Pose:
            print("moving to %s" % pose_to_str(sp))
            mc.Move(h,sp)
            ev = mc.WaitForEvent(h, mc.INFINITE)
            assert(ev.type == mc.EventType.MOVEMENT_FINISHED)



h = None    # device handle
try:
    print_info()

    h = mc.Open("model %s\nlocator %s" % (modelCode, locator))
    print("device opened, handle = %s" % h)

    modelName = mc.GetProperty_s(h,mc.Property.MODEL_NAME)
    print("device model: %s" % modelName)

    mc.SetProperty_i32(h,mc.Property.HOLD_TIME, 1234)
    val = mc.GetProperty_i32(h, mc.Property.HOLD_TIME)
    assert(val == 1234)

    val = mc.GetProperty_f64(h, mc.Property.MAX_SPEED_LINEAR_AXES)    
    mc.SetProperty_f64(h, mc.Property.MAX_SPEED_LINEAR_AXES,val)    
    
    do_referencing = (mc.GetProperty_i32(h, mc.Property.IS_REFERENCED) != mc.TRUE) or force_referencing

    if do_referencing:
        print("referencing device...")
        mc.Reference(h)
        ev = mc.WaitForEvent(h, 100*1000)
        assert(ev.type == mc.EventType.MOVEMENT_FINISHED)
        print("referencing finished.")

    pHome = Pose(0, 0, 0, 0, 0, 0)
    commands = [
        Velocity( 1e-3, 10.0 ),
        pHome,
        Delay(500),
        Pose(1e-3, 1e-3, 1e-3, 0, 0, 0),
        Delay(500),
        Velocity( 2e-3, 10.0 ),
        Pose(-1e-3, -1e-3, -1e-3, 0, 0, 0),
        Delay(500),
        Velocity( 3e-3, 10.0 ),
        Pose(1e-3, 1e-3, 1e-3, 0, 0, 0),
        Delay(1000),
        pHome,
    ]

    exec_commandlist(h,commands)

except mc.Error as err:
    print("SMARACTMC ERROR: %s -> %s (%s) " % (err.func, mc.ErrorCode(err.code), err.code))
    
finally:
    if h != None:
        print("stopping")
        mc.Stop(h)
        print("closing device, handle = %s" % h)
        mc.Close(h)
