/***********************************************************************************

 SmarAct Motion Control code example that demonstrates device initialization
 and performs a few moves.
    
 2019, SmarAct GmbH

************************************************************************************/

#include "SmarActMC.h"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>



using namespace std;

// exits program with return value 1 if ST is != SA_MC_ERROR_NONE
#define EXIT_IF_MC_ERROR(ST) \
{ if((ST) != SA_MC_ERROR_NONE) { \
    cout << "error: " << ResultToString(ST) << " (0x" << hex << (ST) << ")" << endl; \
    return 1; } \
}


// returns status res as a string
string ResultToString(SA_MC_Result res)
{
    const char *info;
    SA_MC_GetResultInfo(res,&info);
    return string(info);
}


// helper for waiting for completion-of-movement events.
// returns the result of the move operation, or the result
// or SA_MC_WaitForEvent is it returns with an error. 
SA_MC_Result WaitForMoveCompletion(SA_MC_Handle h, int32_t timeout)
{
    while(true) {
        SA_MC_Event ev;
        SA_MC_Result res = SA_MC_WaitForEvent(h,&ev,timeout);
        if(res != SA_MC_ERROR_NONE) {
            return res;
        }
        if(ev.type == SA_MC_EVENT_MOVEMENT_FINISHED) {
            return ev.i32;
        }
    }
    return SA_MC_ERROR_NONE;
}


// helper for automatic closing of connection
struct DeviceGuard {
    DeviceGuard(SA_MC_Handle h) : handle(h) {}
    ~DeviceGuard() { SA_MC_Close(handle); }

    SA_MC_Handle handle;
};


int main() 
{
    const string locator = "network:sn:MCS2-00001602"s;
    const unsigned int model = 22000;       // tilt stage
    const double speedLinearAxes = 1e-3;    // 1 mm/s
    const double speedRotaryAxes = 5.;      // 5 degrees/s
    const bool doReferencing = false;


    // open the device

    string options = "model "s + to_string(model) + "\n"
                     "locator "s + locator;
    SA_MC_Handle hDevice = SA_MC_NO_HANDLE;
    SA_MC_Result res = SA_MC_Open(&hDevice,options.c_str());
    EXIT_IF_MC_ERROR(res);

    DeviceGuard dguard(hDevice);  // handles automatic closing of device

    // reference the axes if doReferencing is true

    if(doReferencing) {
        res = SA_MC_Reference(hDevice);
        EXIT_IF_MC_ERROR(res);

        // block until referencing finished or aborted

        res = WaitForMoveCompletion(hDevice,SA_MC_INFINITE);
        EXIT_IF_MC_ERROR(res);
    }

    // set up the axes speed

    res = SA_MC_SetProperty_f64(hDevice,SA_MC_PKEY_MAX_SPEED_LINEAR_AXES,speedLinearAxes);
    EXIT_IF_MC_ERROR(res);

    res = SA_MC_SetProperty_f64(hDevice,SA_MC_PKEY_MAX_SPEED_ROTARY_AXES,speedRotaryAxes);
    EXIT_IF_MC_ERROR(res);

    // move to the zero pose

    SA_MC_Pose pose = {0.,0.,0.,0.,0.,0}; // zero pose = home position

    res = SA_MC_Move(hDevice,&pose);
    EXIT_IF_MC_ERROR(res);
    res = WaitForMoveCompletion(hDevice,SA_MC_INFINITE);
    EXIT_IF_MC_ERROR(res);

    // move to a different pose and wait for completion

    pose.rx = 5.;
    res = SA_MC_Move(hDevice,&pose);
    EXIT_IF_MC_ERROR(res);
    res = WaitForMoveCompletion(hDevice,SA_MC_INFINITE);
    EXIT_IF_MC_ERROR(res);

    // move to a different pose

    pose.x = -3e-3;
    pose.rx = -5.;
    res = SA_MC_Move(hDevice,&pose);
    EXIT_IF_MC_ERROR(res);
    res = WaitForMoveCompletion(hDevice,SA_MC_INFINITE);
    EXIT_IF_MC_ERROR(res);


    // move back to zero
    pose = {0.,0.,0.,0.,0.,0}; 
    res = SA_MC_Move(hDevice,&pose);
    EXIT_IF_MC_ERROR(res);
    res = WaitForMoveCompletion(hDevice,SA_MC_INFINITE);
    EXIT_IF_MC_ERROR(res);


    // move to Y = 3 mm and don't block this thread until completion...

    pose.y = 3e-3;
    res = SA_MC_Move(hDevice,&pose);
    EXIT_IF_MC_ERROR(res);

    // the movement will take a few seconds.
    // wait 500 milliseconds, then stop

    this_thread::sleep_for(500ms);

    res = SA_MC_Stop(hDevice);
    EXIT_IF_MC_ERROR(res);

    // read and print the current pose

    res = SA_MC_GetPose(hDevice,&pose);
    EXIT_IF_MC_ERROR(res);

    cout << "current pose = \n  " 
         << pose.x << "\n  "
         << pose.y << "\n  "
         << pose.z << "\n  "
         << pose.rx << "\n  "
         << pose.ry << "\n  "
         << pose.rz << endl;

    // device connection is closed by DeviceGuard...
    return 0;
}

