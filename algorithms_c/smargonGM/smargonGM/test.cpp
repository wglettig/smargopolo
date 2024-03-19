#include <stdio.h>
#include "smargon.h"

int main (int argc, char * argv[]) {

	smargon MySmarGon;
	char filename[] = "smargonparameters.txt";
	
	if (MySmarGon.readParamFile(filename)!=0){
		printf("Could not open 'smargonparameters.txt'. Using Default Values.\n");
	} else {
		printf("'smargonparameters.txt' loaded.Ready...\n");
	}

	double UCS[10] = {0., 0.,0.,22.0e-3,  0.,0.,0., 10e-3,-10e-3,188e-3};
	double MCS[10] = {0., 0.,0.,0., 0.,0.,0., 0.,0.,0.};
	int err;
	char msg[256];
	err = MySmarGon.IK(UCS, MCS, msg);
	printf("err: %d, String: %s\n", err, msg);
	printf("UCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", UCS[1],UCS[2],UCS[3],UCS[4],UCS[5],UCS[6],UCS[7],UCS[8],UCS[9]);
	printf("MCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", MCS[1],MCS[2],MCS[3],MCS[4],MCS[5],MCS[6],MCS[7],MCS[8],MCS[9]);
	err = MySmarGon.FK(MCS, UCS, msg);
	printf("err: %d, String: %s\n", err, msg);
	printf("MCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", MCS[1],MCS[2],MCS[3],MCS[4],MCS[5],MCS[6],MCS[7],MCS[8],MCS[9]);
	printf("UCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", UCS[1],UCS[2],UCS[3],UCS[4],UCS[5],UCS[6],UCS[7],UCS[8],UCS[9]);

}

