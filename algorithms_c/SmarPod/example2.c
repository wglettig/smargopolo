/*
* Copyright (c) 2013 SmarAct GmbH
*
* Programming example for the SmarPod C/C++ API.
* This example demostrates how to use the Smarpod_ConfigureSystem command.
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


const unsigned int cSmarpodModel = 10001;


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
    unsigned int id;
    /* initialize the SmarPod at the first USB MCS */
    Smarpod_Status st = Smarpod_Open(&id,cSmarpodModel,"usb:ix:0","");           
    if(st != SMARPOD_OK && st != SMARPOD_SYSTEM_CONFIGURATION_ERROR)
    {
        /* the result is neither OK nor an invalid system config => exit */
        ExitOnError(st);
    }

    /* at this point, the Initialization status of SmarPod 0 is either OK
       or SMARPOD_SYSTEM_CONFIGURATION_ERROR. Smarpod_ConfigureSystem
       can be called for SmarPod 0. */


    /* Smarpod_ConfigureSystem should be called once after installing
       a new firmware on the MCS controller. */
    ExitOnError( Smarpod_ConfigureSystem(id) );

    /* after Smarpod_ConfigureSystem the sensors must be re-calibrated */
    ExitOnError( Smarpod_Calibrate(id) );        

    LogError( Smarpod_Close(id) );
    return 0;
}

