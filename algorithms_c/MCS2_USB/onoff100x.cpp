//SmarAct SDK Includes:
#include <SmarActControl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctime>
unsigned int milliseconds = 1000;

void verbose(SA_CTL_Result_t result) {
    // The return value indicates if the call was successful (SA_CTL_ERROR_NONE)
    // or if an error occurred.
    if (result==SA_CTL_ERROR_NONE){
        printf("OK.\n");
    } else {
        printf("Error %d\n",result);
    }
    return;
}

int main(int argc, char **argv)
{
    // Set Up SmarAct MCS2 Controller: /////////////////////////////////////////
    const char *version = SA_CTL_GetFullVersionString();
    printf("SmarActCTL library version: %s.", version);

    char locator[] = { "usb:ix:0" }; //USB
    //char locator[] = { "192.168.1.200:55550" }; //Ethernet

    char config[] = {""}; // Additional configuration parameters (unused for now)

    // All SA_CTL functions return a result code of the type "SA_CTL_Result_t".
    SA_CTL_Result_t result;
    int64_t rbv_start;
    int64_t rbv_back;
    int64_t rbv_off;
    int64_t rbv_diff;
    int64_t diff_back;
    int64_t diff_sum=0;
    double diff_avg;
    int64_t reltarget;
    srand((unsigned)time(0));

    SA_CTL_DeviceHandle_t dHandle; //Device handle for MCS2 device
    // Try to open the MCS2 device
    printf("SA_CTL_Open: ");
    verbose(SA_CTL_Open(&dHandle, locator, config));
    
    printf("SA_CTL_Set Move Relative: ");
    verbose(SA_CTL_SetProperty_i32(dHandle, 2, SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_RELATIVE));
    
    for (int i=0;i<10;i++) {
        reltarget = -25000000+(rand()%50000000); //Generate relative distance to move (Random [-25,25]um)
        printf("SA_CTL_ENABLE:");
        verbose(SA_CTL_SetProperty_i32(dHandle, 2, SA_CTL_PKEY_AMPLIFIER_ENABLED, SA_CTL_ENABLED));
	usleep(500 * milliseconds);
	verbose(SA_CTL_GetProperty_i64(dHandle, 2, SA_CTL_PKEY_POSITION, &rbv_start, 0));
        printf("POSITION START: %ld\n", rbv_start);
	printf("SA_CTL_Move Forward: %ld [nm]: ",reltarget/1000 );
        verbose(SA_CTL_Move(dHandle, 2, reltarget, 0)); //Move Relative POS
	usleep(500 * milliseconds);
	printf("SA_CTL_Move Back: %ld [nm]: ",-reltarget/1000 );
        verbose(SA_CTL_Move(dHandle, 2, -reltarget, 0)); //Move Relative POS
	usleep(500 * milliseconds);
	verbose(SA_CTL_GetProperty_i64(dHandle, 2, SA_CTL_PKEY_POSITION, &rbv_back, 0));
        printf("POSITION BACK: %ld\n", rbv_back);
        printf("SA_CTL_DISABLE:");
        verbose(SA_CTL_SetProperty_i32(dHandle, 2, SA_CTL_PKEY_AMPLIFIER_ENABLED, SA_CTL_DISABLED));
	usleep(500 * milliseconds);
	verbose(SA_CTL_GetProperty_i64(dHandle, 2, SA_CTL_PKEY_POSITION, &rbv_off, 0));
        printf("POSITION OFF: %ld\n", rbv_off);
	rbv_diff=rbv_off-rbv_start;
	diff_sum+=rbv_diff;
        diff_back=rbv_back-rbv_start;
        diff_avg=diff_sum/(i+1);
	printf("                                  DIFFERENCE BACK %ld\n", diff_back);
	printf("                                  DIFFERENCE OFF  %ld\n", rbv_diff);
	printf("                                  AVERAGE    %f\n", diff_avg);

	
	usleep(100 * milliseconds);
    }	   
	   
    verbose(SA_CTL_Close(dHandle));
}
