/*
* Copyright (c) 2013 SmarAct GmbH
*
* Programming example for the SmarPod C/C++ API.
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
*/


#include <stdlib.h>
#include <stdio.h>
#include <SmarPod.h>


const int cCalibrate = 0;
const int cEnableAccelerationControl = 0;
const double cAcceleration = 0.001;



int LogError(Smarpod_Status status)
{
    if(status != SMARPOD_OK)
    {
        const char *info;
        if(Smarpod_GetStatusInfo(status,&info))
            printf("unknown SmarPod status\n");
        else
            printf("error: %s\n",info);
    }
    return status;
}


void ExitOnError(Smarpod_Status status)
{
    if(LogError(status))
        exit(1);
}


int main()
{
//    const double kXyMax = 0.0102;
    const double kXyMax = 0.001;
    const Smarpod_Pose pZero = { 0.0,0.0,0.0, 0.0,0.0,0.0 };
    const Smarpod_Pose pZero1 = { 0.0,-kXyMax,0.0, 0.0,0.0,0.0 };
    const Smarpod_Pose pZ0Left  = { -kXyMax,-kXyMax,0.0, 0.0,0.0,0.0 };
    const Smarpod_Pose pZ0Right = {  kXyMax,-kXyMax,0.0, 0.0,0.0,0.0 };
    const Smarpod_Pose pZ1Left  = { -kXyMax, kXyMax,0.0, 0.0,0.0,0.0 };
    const Smarpod_Pose pZ1Right = {  kXyMax, kXyMax,0.0, 0.0,0.0,0.0 };

    /* replace with the code for YOUR SmarPod model: */
    unsigned int model = 10068;    

    unsigned int major,minor,update;
    int referenced = 0;
    Smarpod_Pose pose;
    unsigned int mstatus;
    unsigned int id;

    ExitOnError( Smarpod_GetDLLVersion(&major,&minor,&update) );
    printf("using SmarPod library version %u.%u.%u\n",major,minor,update);

 //   ExitOnError( Smarpod_Open(&id,model,"usb:ix:0","") );           /* initialize the SmarPod at the first USB MCS */
    ExitOnError( Smarpod_Open(&id,model,"network:192.168.1.123","") );           /* initialize the SmarPod at the first USB MCS */
    ExitOnError( Smarpod_SetSensorMode(id,SMARPOD_SENSORS_ENABLED)); /* enable the sensors */
    if(cCalibrate)                                                  /* calibrate the sensors. this has to be done only once */
        ExitOnError( Smarpod_Calibrate(id) );                        /* if the sensors are calibrated, this step can be skipped */

    ExitOnError( Smarpod_IsReferenced(id,&referenced) );             /* check if the actuators know their absolute positions */
    if(!referenced)
        ExitOnError( Smarpod_FindReferenceMarks(id) );               /* ... if not, find the reference-marks */

    if(cEnableAccelerationControl)                                  /* optionally set a maximum acceleration (see SmarPod Programmer's Guide) */
        LogError( Smarpod_SetAcceleration(id,1,cAcceleration) );

    LogError( Smarpod_SetSpeed(id,1,0.01) );
    LogError( Smarpod_Move(id,&pZero,SMARPOD_HOLDTIME_INFINITE,1) );
    printf("zero\n");
    LogError( Smarpod_SetSpeed(id,1,0.01) );
    LogError( Smarpod_Move(id,&pZero1,SMARPOD_HOLDTIME_INFINITE,1) );
    printf("zero\n");
    LogError( Smarpod_SetSpeed(id,1,0.005) );                        /* set the movement speed to 2mm/sec */
    LogError( Smarpod_Move(id,&pZ0Left,SMARPOD_HOLDTIME_INFINITE,1) ); /* move and wait until movement has finished */
    printf("0right\n");
    LogError( Smarpod_SetSpeed(id,1,0.005) );                        /* set the movement speed to 2mm/sec */
    LogError( Smarpod_Move(id,&pZ1Left,SMARPOD_HOLDTIME_INFINITE,1) ); /* move and wait until movement has finished */
    printf("1left\n");
    LogError( Smarpod_SetSpeed(id,1,0.005) );
    LogError( Smarpod_Move(id,&pZ1Right,SMARPOD_HOLDTIME_INFINITE,1) );
    printf("1right\n");
    LogError( Smarpod_SetSpeed(id,1,0.005) );                        /* set the movement speed to 2mm/sec */
    LogError( Smarpod_Move(id,&pZ0Right,SMARPOD_HOLDTIME_INFINITE,1) ); /* move and wait until movement has finished */
    printf("0left\n");
    LogError( Smarpod_SetSpeed(id,1,0.01) );
    LogError( Smarpod_Move(id,&pZero1,SMARPOD_HOLDTIME_INFINITE,1) );
    printf("zero\n");
    LogError( Smarpod_SetSpeed(id,1,0.01) );
    LogError( Smarpod_Move(id,&pZero,SMARPOD_HOLDTIME_INFINITE,1) );
    printf("zero\n");

    LogError( Smarpod_GetPose(id,&pose) );
    printf("pose = (%lf,%lf,%lf,%lf,%lf,%lf)\n",pose.positionX,pose.positionY,
        pose.positionZ,pose.rotationX,pose.rotationY,pose.rotationZ);
    LogError( Smarpod_GetMoveStatus(id,&mstatus) );
    printf("move-status = %u\n",mstatus);

    LogError( Smarpod_Close(id) );                                  /* release SmarPod */
    return 0;
}

