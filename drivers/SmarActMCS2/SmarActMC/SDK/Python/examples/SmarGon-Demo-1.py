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

model = 31001           # replace by model code of your positioner
locator = "usb:ix:0"    # replace by locator string of your controller
force_referencing = False


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


def move_sequence(h,seq):
    """
    executes command sequence seq.
    commands can be objects of type Delay, Velocity and Pose
    """
    for sp in seq:
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

    h = mc.Open("model %s\nlocator %s" % (model, locator))
    print("device opened, handle = %s" % h)

    mc.SetProperty_i32(h,mc.Property.HOLD_TIME, 1234)
    val = mc.GetProperty_i32(h, mc.Property.HOLD_TIME)
    assert(val == 1234)

    val = mc.GetProperty_f64(h, mc.Property.MAX_SPEED_LINEAR_AXES)    
    mc.SetProperty_f64(h, mc.Property.MAX_SPEED_LINEAR_AXES,val)    
    
    do_referencing = (mc.GetProperty_i32(h, mc.Property.IS_REFERENCED) != mc.TRUE)

    if do_referencing or force_referencing:
        print("referencing...")
        mc.Reference(h)
        ev = mc.WaitForEvent(h, 100*1000)
        assert(ev.type == mc.EventType.MOVEMENT_FINISHED)
        print("referencing finished.")

    pHome = Pose(0, 0, 0, 0, 0, 0)
    pSequence = [
        pHome,
        Delay(500),

        Pose(0, 0, 0.002, 0, 0, 0),
        Delay(500),
        Velocity( 1e-3, 10.0 ),
        Pose(0, 0, -0.002, 0, 0, 0),
        Pose(-0.002, 0, 0, 0, 0, 0),
        Delay(500),
        Velocity( 3e-3, 45.0 ),
        Pose(0, 0, 0, 0, 0, 45),
        Pose(0, 0, 0, 0, 45, 45),
        Delay(500),
        Velocity( 7e-3, 60.0 ),
        Pose(0, 3e-3, 0, 0, -45, 90),
        Delay(1000),
        pHome,
        Delay(500),

        Pose(-3e-3, 0, -2e-3, 0, 0, 0),
        Delay(200),
        Pose(20e-3, -20e-3, 2e-3, 0, 0, 0),
        Delay(1000),

        pHome,
    ]

    move_sequence(h,pSequence)

except mc.Error as err:
    print("SMARACTMC ERROR: %s -> %s (%s) " % (err.func, mc.ErrorCode(err.code), err.code))
    
finally:
    if h != None:
        print("stopping")
        mc.Stop(h)
        print("closing device, handle = %s" % h)
        mc.Close(h)
