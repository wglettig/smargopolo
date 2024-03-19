#include <stdio.h>
#include "prigo.h"

int main (int argc, char * argv[]) {

	prigo * MyPrigo;

	MyPrigo = new prigo();
	char filename[] = "prigoparameters.txt";
	if (MyPrigo->readParamFile(filename)!=0){
		printf("Could not open 'prigoparameters.txt'. Using Default Values.\n");
	} else {
		printf("'prigoparameters.txt' loaded.Ready...\n");
	}

	double UCS[10] = {0.,  0.,0.,20., 0.,0.,0., 0.,0.,275.};
	double MCS[10] = {0.,  0.,0.,0., 0.,0.,0., 0.,0.,300.};
	int err;
	char msg[256];
	err = MyPrigo->mgi(UCS, MCS, msg);
	printf("err: %d, String: %s\n", err, msg);
	printf("UCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", UCS[1],UCS[2],UCS[3],UCS[4],UCS[5],UCS[6],UCS[7],UCS[8],UCS[9]);
	printf("MCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", MCS[1],MCS[2],MCS[3],MCS[4],MCS[5],MCS[6],MCS[7],MCS[8],MCS[9]);
	err = MyPrigo->mgd(MCS, UCS, msg);
	printf("err: %d, String: %s\n", err, msg);
	printf("MCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", MCS[1],MCS[2],MCS[3],MCS[4],MCS[5],MCS[6],MCS[7],MCS[8],MCS[9]);
	printf("UCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", UCS[1],UCS[2],UCS[3],UCS[4],UCS[5],UCS[6],UCS[7],UCS[8],UCS[9]);

/*{	printf("NEW RUN:\n");
	double UCS[10] = {0.,  0.,0.,16., 45.,0.,0., 0.,0.,300.};
	double MCS[10];
	printf("UCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", UCS[1],UCS[2],UCS[3],UCS[4],UCS[5],UCS[6],UCS[7],UCS[8],UCS[9]);

	if (MyPrigo->readParamFile("prigoparameters.txt")!=0){
		printf("Could not open 'prigoparameters.txt'. Using Default Values.\n");
	} else {
		printf("'prigoparameters.txt' loaded.Ready...\n");
	}
	MyPrigo->mgi(UCS, MCS);
	printf("MCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", MCS[1],MCS[2],MCS[3],MCS[4],MCS[5],MCS[6],MCS[7],MCS[8],MCS[9]);
	MyPrigo->mgd(MCS, UCS);
	printf("UCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", UCS[1],UCS[2],UCS[3],UCS[4],UCS[5],UCS[6],UCS[7],UCS[8],UCS[9]);
}*/	
	delete MyPrigo;
}

