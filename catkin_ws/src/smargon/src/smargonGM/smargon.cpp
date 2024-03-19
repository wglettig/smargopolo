// smargon.cpp: implementation of the smargon class.
// Wayne Glettig 13.9.2020
//////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <string.h>

#include "math.h"
#include "MatriceVectLib.h"  //LSRO Matrix library (wayne erweitert)
#include "smargon.h"

//SWITCH TO ENABLE TESTING
#define TESTING

#ifdef TESTING
double Var_MG[6][5], Var_MGI[6][5];
#else
extern double Var_MG[6][5], Var_MGI[6][5];
#endif
//

//sgn function (used in traj)
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
smargon::smargon()
{
    //initialize lengths and distances:
    //all lengths in [mm]
    l01 = 50.5;
    l11 = 25.0 - (17.0)/2; //half distance between sliders table midline
    l12 = l11;
    l21 = l11;
    l22 = l11;
    l23 = 13.5; //Distance between q1 & q2 stage level
    l31 = 11.5; //Distance from q3 table to middle of red part
    l32 = 68.5 - (80.0)/2;
    l33 = l31;
    l34 = l32;
    l41 = 76.5;
    l42 = 25.5;
    l51 = 10.0;
    l52 = 2.5;
    l61 = 64.422; //Connecting rod length
    l71 = 5.0;  //Swing dimensions
    l72 = 17.67;
    l73 = 5.2;
    l74 = 9;//old:1.53;
    
    //offsets Sliders:
    offsetQ1 = 0.00;
    offsetQ2 = 0.00;
    offsetQ3 = 0.00;
    offsetQ4 = 0.00;
    offsetQ5 = 0.00;
    offsetQ6 = 0.00;

    //start values:
    SHX = 0;
    SHY = 0;
    SHZ = 20;

    OX = 0;
    OY = 0;
    OZ = 190;

    CHI = 0;
    PHI = 0;
    OMEGA = 0;

}

smargon::~smargon()
{

}

int smargon::readParamFile(char* filename)
{	
	//define variables
	FILE * pFile;
	char mode[4]="r"; //mode read
	char read_str[1000]; //string buffer
	double read_double = 0.; //double buffer
	//open file:
	pFile = fopen(filename, mode);
	//check if file can be opened:
	if (pFile==NULL){
		return 1; //error reading file
	} else {
		//start reading line by line:
		while (!feof(pFile)) {
			//fgets(linebuffer, 100, pFile); //read a line in.
			fscanf (pFile, "%s\t%lf\n", read_str, &read_double);
			if      (strcmp(read_str, "l01")==0){ l01=read_double;printf("l01\t%lf\n",read_double);}
			else if (strcmp(read_str, "l11")==0){ l11=read_double;printf("l11\t%lf\n",read_double);}
			else if (strcmp(read_str, "l12")==0){ l12=read_double;printf("l12\t%lf\n",read_double);}
			else if (strcmp(read_str, "l21")==0){ l21=read_double;printf("l21\t%lf\n",read_double);}
			else if (strcmp(read_str, "l22")==0){ l22=read_double;printf("l22\t%lf\n",read_double);}
			else if (strcmp(read_str, "l23")==0){ l23=read_double;printf("l23\t%lf\n",read_double);}
			else if (strcmp(read_str, "l31")==0){ l31=read_double;printf("l31\t%lf\n",read_double);}
			else if (strcmp(read_str, "l32")==0){ l32=read_double;printf("l32\t%lf\n",read_double);}
			else if (strcmp(read_str, "l33")==0){ l33=read_double;printf("l33\t%lf\n",read_double);}
			else if (strcmp(read_str, "l34")==0){ l34=read_double;printf("l34\t%lf\n",read_double);}
			else if (strcmp(read_str, "l41")==0){ l41=read_double;printf("l41\t%lf\n",read_double);}
			else if (strcmp(read_str, "l42")==0){ l42=read_double;printf("l42\t%lf\n",read_double);}
			else if (strcmp(read_str, "l51")==0){ l51=read_double;printf("l51\t%lf\n",read_double);}
			else if (strcmp(read_str, "l52")==0){ l52=read_double;printf("l52\t%lf\n",read_double);}
			else if (strcmp(read_str, "l61")==0){ l61=read_double;printf("l61\t%lf\n",read_double);}
			else if (strcmp(read_str, "l71")==0){ l71=read_double;printf("l71\t%lf\n",read_double);}
			else if (strcmp(read_str, "l72")==0){ l72=read_double;printf("l72\t%lf\n",read_double);}
			else if (strcmp(read_str, "l73")==0){ l73=read_double;printf("l73\t%lf\n",read_double);}
			else if (strcmp(read_str, "l74")==0){ l74=read_double;printf("l74\t%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ1")==0){ offsetQ1=read_double;printf("offsetQ1\t%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ2")==0){ offsetQ2=read_double;printf("offsetQ2\t%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ3")==0){ offsetQ3=read_double;printf("offsetQ3\t%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ4")==0){ offsetQ4=read_double;printf("offsetQ4\t%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ5")==0){ offsetQ5=read_double;printf("offsetQ5\t%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ6")==0){ offsetQ6=read_double;printf("offsetQ6\t%lf\n",read_double);}

			else if (strcmp(read_str, "q1_start" )==0) { q1_start =read_double;printf("q1_start\t%lf\n", q1_start );}
			else if (strcmp(read_str, "q1_limpos")==0) { q1_limpos=read_double;printf("q1_limpos\t%lf\n",q1_limpos);}
			else if (strcmp(read_str, "q1_limneg")==0) { q1_limneg=read_double;printf("q1_limneg\t%lf\n",q1_limneg);}
			else if (strcmp(read_str, "q2_start" )==0) { q2_start =read_double;printf("q2_start\t%lf\n", q2_start );}
			else if (strcmp(read_str, "q2_limpos")==0) { q2_limpos=read_double;printf("q2_limpos\t%lf\n",q2_limpos);}
			else if (strcmp(read_str, "q2_limneg")==0) { q2_limneg=read_double;printf("q2_limneg\t%lf\n",q2_limneg);}
			else if (strcmp(read_str, "q3_start" )==0) { q3_start =read_double;printf("q3_start\t%lf\n", q3_start );}
			else if (strcmp(read_str, "q3_limpos")==0) { q3_limpos=read_double;printf("q3_limpos\t%lf\n",q3_limpos);}
			else if (strcmp(read_str, "q3_limneg")==0) { q3_limneg=read_double;printf("q3_limneg\t%lf\n",q3_limneg);}
			else if (strcmp(read_str, "q4_start" )==0) { q4_start =read_double;printf("q4_start\t%lf\n", q4_start );}
			else if (strcmp(read_str, "q4_limpos")==0) { q4_limpos=read_double;printf("q4_limpos\t%lf\n",q4_limpos);}
			else if (strcmp(read_str, "q4_limneg")==0) { q4_limneg=read_double;printf("q4_limneg\t%lf\n",q4_limneg);}
			else if (strcmp(read_str, "q5_start" )==0) { q5_start =read_double;printf("q5_start\t%lf\n", q5_start );}
			else if (strcmp(read_str, "q5_limpos")==0) { q5_limpos=read_double;printf("q5_limpos\t%lf\n",q5_limpos);}
			else if (strcmp(read_str, "q5_limneg")==0) { q5_limneg=read_double;printf("q5_limneg\t%lf\n",q5_limneg);}
			else if (strcmp(read_str, "q6_start" )==0) { q6_start =read_double;printf("q6_start\t%lf\n", q6_start );}
			else if (strcmp(read_str, "q6_limpos")==0) { q6_limpos=read_double;printf("q6_limpos\t%lf\n",q6_limpos);}
			else if (strcmp(read_str, "q6_limneg")==0) { q6_limneg=read_double;printf("q6_limneg\t%lf\n",q6_limneg);}
			
			else if (strcmp(read_str, "SHX_start" )==0) { SHX_start =read_double;printf("SHX_start\t%lf\n", SHX_start );}
			else if (strcmp(read_str, "SHX_limpos")==0) { SHX_limpos=read_double;printf("SHX_limpos\t%lf\n",SHX_limpos);}
			else if (strcmp(read_str, "SHX_limneg")==0) { SHX_limneg=read_double;printf("SHX_limneg\t%lf\n",SHX_limneg);}
			else if (strcmp(read_str, "SHY_start" )==0) { SHY_start =read_double;printf("SHY_start\t%lf\n", SHY_start );}
			else if (strcmp(read_str, "SHY_limpos")==0) { SHY_limpos=read_double;printf("SHY_limpos\t%lf\n",SHY_limpos);}
			else if (strcmp(read_str, "SHY_limneg")==0) { SHY_limneg=read_double;printf("SHY_limneg\t%lf\n",SHY_limneg);}
			else if (strcmp(read_str, "SHZ_start" )==0) { SHZ_start =read_double;printf("SHZ_start\t%lf\n", SHZ_start );}
			else if (strcmp(read_str, "SHZ_limpos")==0) { SHZ_limpos=read_double;printf("SHZ_limpos\t%lf\n",SHZ_limpos);}
			else if (strcmp(read_str, "SHZ_limneg")==0) { SHZ_limneg=read_double;printf("SHZ_limneg\t%lf\n",SHZ_limneg);}
			else if (strcmp(read_str, "OMEGA_start" )==0) { OMEGA_start =read_double;printf("OMEGA_start\t%lf\n", OMEGA_start );}
			else if (strcmp(read_str, "OMEGA_limpos")==0) { OMEGA_limpos=read_double;printf("OMEGA_limpos\t%lf\n",OMEGA_limpos);}
			else if (strcmp(read_str, "OMEGA_limneg")==0) { OMEGA_limneg=read_double;printf("OMEGA_limneg\t%lf\n",OMEGA_limneg);}
			else if (strcmp(read_str, "CHI_start" )==0) { CHI_start =read_double;printf("CHI_start\t%lf\n", CHI_start );}
			else if (strcmp(read_str, "CHI_limpos")==0) { CHI_limpos=read_double;printf("CHI_limpos\t%lf\n",CHI_limpos);}
			else if (strcmp(read_str, "CHI_limneg")==0) { CHI_limneg=read_double;printf("CHI_limneg\t%lf\n",CHI_limneg);}
			else if (strcmp(read_str, "PHI_start" )==0) { PHI_start =read_double;printf("PHI_start\t%lf\n", PHI_start );}
			else if (strcmp(read_str, "PHI_limpos")==0) { PHI_limpos=read_double;printf("PHI_limpos\t%lf\n",PHI_limpos);}
			else if (strcmp(read_str, "PHI_limneg")==0) { PHI_limneg=read_double;printf("PHI_limneg\t%lf\n",PHI_limneg);}
			else if (strcmp(read_str, "OX_start"  )==0) { OX_start  =read_double;printf("OX_start\t%lf\n", OX_start );}
			else if (strcmp(read_str, "OX_limpos" )==0) { OX_limpos =read_double;printf("OX_limpos\t%lf\n",OX_limpos);}
			else if (strcmp(read_str, "OX_limneg" )==0) { OX_limneg =read_double;printf("OX_limneg\t%lf\n",OX_limneg);}
			else if (strcmp(read_str, "OY_start"  )==0) { OY_start  =read_double;printf("OY_start\t%lf\n", OY_start );}
			else if (strcmp(read_str, "OY_limpos" )==0) { OY_limpos =read_double;printf("OY_limpos\t%lf\n",OY_limpos);}
			else if (strcmp(read_str, "OY_limneg" )==0) { OY_limneg =read_double;printf("OY_limneg\t%lf\n",OY_limneg);}
			else if (strcmp(read_str, "OZ_start"  )==0) { OZ_start  =read_double;printf("OZ_start\t%lf\n", OZ_start );}
			else if (strcmp(read_str, "OZ_limpos" )==0) { OZ_limpos =read_double;printf("OZ_limpos\t%lf\n",OZ_limpos);}
			else if (strcmp(read_str, "OZ_limneg" )==0) { OZ_limneg =read_double;printf("OZ_limneg\t%lf\n",OZ_limneg);}
                                                                                 
			else if (strcmp(read_str, "BX_start"     )==0) { BX_start     =read_double;printf("BX_start\t%lf\n", BX_start );}
			else if (strcmp(read_str, "BX_limpos"    )==0) { BX_limpos    =read_double;printf("BX_limpos\t%lf\n",BX_limpos);}
			else if (strcmp(read_str, "BX_limneg"    )==0) { BX_limneg    =read_double;printf("BX_limneg\t%lf\n",BX_limneg);}
			else if (strcmp(read_str, "BY_start"     )==0) { BY_start     =read_double;printf("BY_start\t%lf\n", BY_start );}
			else if (strcmp(read_str, "BY_limpos"    )==0) { BY_limpos    =read_double;printf("BY_limpos\t%lf\n",BY_limpos);}
			else if (strcmp(read_str, "BY_limneg"    )==0) { BY_limneg    =read_double;printf("BY_limneg\t%lf\n",BY_limneg);}
			else if (strcmp(read_str, "BOMEGA_start" )==0) { BOMEGA_start =read_double;printf("BOMEGA_start\t%lf\n", BOMEGA_start );}
			else if (strcmp(read_str, "BOMEGA_limpos")==0) { BOMEGA_limpos=read_double;printf("BOMEGA_limpos\t%lf\n",BOMEGA_limpos);}
			else if (strcmp(read_str, "BOMEGA_limneg")==0) { BOMEGA_limneg=read_double;printf("BOMEGA_limneg\t%lf\n",BOMEGA_limneg);}
			else if (strcmp(read_str, "BCHI_start"   )==0) { BCHI_start   =read_double;printf("BCHI_start\t%lf\n", BCHI_start );}
			else if (strcmp(read_str, "BCHI_limpos"  )==0) { BCHI_limpos  =read_double;printf("BCHI_limpos\t%lf\n",BCHI_limpos);}
			else if (strcmp(read_str, "BCHI_limneg"  )==0) { BCHI_limneg  =read_double;printf("BCHI_limneg\t%lf\n",BCHI_limneg);}
			else if (strcmp(read_str, "BPHI_start"   )==0) { BPHI_start   =read_double;printf("BPHI_start\t%lf\n", BPHI_start );}
			else if (strcmp(read_str, "BPHI_limpos"  )==0) { BPHI_limpos  =read_double;printf("BPHI_limpos\t%lf\n",BPHI_limpos);}
			else if (strcmp(read_str, "BPHI_limneg"  )==0) { BPHI_limneg  =read_double;printf("BPHI_limneg\t%lf\n",BPHI_limneg);}   

		}
		fclose(pFile);
	}
	return 0;
}

int smargon::writeParamFile(char* filename)
{	
	//define variables
	FILE * pFile;
	char mode[4]="w"; //mode write
	//open file:
	pFile = fopen(filename, mode);
	//check if file can be opened:
	if (pFile==NULL){
		return 1; //error reading file
		printf("Error opeing file: %s to write.\n",filename);
	} else {
		//start writing line by line:
		fprintf(pFile, "l01\t%lf\n",l01);
		fprintf(pFile, "l11\t%lf\n",l11);
		fprintf(pFile, "l12\t%lf\n",l12);
		fprintf(pFile, "l21\t%lf\n",l21);
		fprintf(pFile, "l22\t%lf\n",l22);
		fprintf(pFile, "l23\t%lf\n",l23);
		fprintf(pFile, "l31\t%lf\n",l31);
		fprintf(pFile, "l32\t%lf\n",l32);
		fprintf(pFile, "l33\t%lf\n",l33);
		fprintf(pFile, "l34\t%lf\n",l34);
		fprintf(pFile, "l41\t%lf\n",l41);
		fprintf(pFile, "l42\t%lf\n",l42);
		fprintf(pFile, "l51\t%lf\n",l51);
		fprintf(pFile, "l52\t%lf\n",l52);
		fprintf(pFile, "l61\t%lf\n",l61);
		fprintf(pFile, "l71\t%lf\n",l71);
		fprintf(pFile, "l72\t%lf\n",l72);
		fprintf(pFile, "l73\t%lf\n",l73);
		fprintf(pFile, "l74\t%lf\n",l74);
		fprintf(pFile, "offsetQ1\t%lf\n",offsetQ1);
		fprintf(pFile, "offsetQ2\t%lf\n",offsetQ2);
		fprintf(pFile, "offsetQ3\t%lf\n",offsetQ3);
		fprintf(pFile, "offsetQ4\t%lf\n",offsetQ4);
		fprintf(pFile, "offsetQ5\t%lf\n",offsetQ5);
		fprintf(pFile, "offsetQ6\t%lf\n",offsetQ6);
		fprintf(pFile, "\n");

		fprintf(pFile, "q1_start\t%lf\n", q1_start);
		fprintf(pFile, "q1_limpos\t%lf\n",q1_limpos);
		fprintf(pFile, "q1_limneg\t%lf\n",q1_limneg);
		fprintf(pFile, "q2_start\t%lf\n", q2_start);
		fprintf(pFile, "q2_limpos\t%lf\n",q2_limpos);
		fprintf(pFile, "q2_limneg\t%lf\n",q2_limneg);
		fprintf(pFile, "q3_start\t%lf\n", q3_start);
		fprintf(pFile, "q3_limpos\t%lf\n",q3_limpos);
		fprintf(pFile, "q3_limneg\t%lf\n",q3_limneg);	
		fprintf(pFile, "q4_start\t%lf\n", q4_start);
		fprintf(pFile, "q4_limpos\t%lf\n",q4_limpos);
		fprintf(pFile, "q4_limneg\t%lf\n",q4_limneg);
		fprintf(pFile, "q5_start\t%lf\n", q5_start);
		fprintf(pFile, "q5_limpos\t%lf\n",q5_limpos);
		fprintf(pFile, "q5_limneg\t%lf\n",q5_limneg);
		fprintf(pFile, "q6_start\t%lf\n", q6_start);
		fprintf(pFile, "q6_limpos\t%lf\n",q6_limpos);
		fprintf(pFile, "q6_limneg\t%lf\n",q6_limneg);
		fprintf(pFile, "\n");
		
		fprintf(pFile, "SHX_start\t%lf\n", SHX_start);
		fprintf(pFile, "SHX_limpos\t%lf\n",SHX_limpos);
		fprintf(pFile, "SHX_limneg\t%lf\n",SHX_limneg);
		fprintf(pFile, "SHY_start\t%lf\n", SHY_start);
		fprintf(pFile, "SHY_limpos\t%lf\n",SHY_limpos);
		fprintf(pFile, "SHY_limneg\t%lf\n",SHY_limneg);
		fprintf(pFile, "SHZ_start\t%lf\n", SHZ_start);
		fprintf(pFile, "SHZ_limpos\t%lf\n",SHZ_limpos);
		fprintf(pFile, "SHZ_limneg\t%lf\n",SHZ_limneg);
		fprintf(pFile, "OMEGA_start\t%lf\n", OMEGA_start);
		fprintf(pFile, "OMEGA_limpos\t%lf\n",OMEGA_limpos);
		fprintf(pFile, "OMEGA_limneg\t%lf\n",OMEGA_limneg);
		fprintf(pFile, "CHI_start\t%lf\n", CHI_start);
		fprintf(pFile, "CHI_limpos\t%lf\n",CHI_limpos);
		fprintf(pFile, "CHI_limneg\t%lf\n",CHI_limneg);
		fprintf(pFile, "PHI_start\t%lf\n", PHI_start);
		fprintf(pFile, "PHI_limpos\t%lf\n",PHI_limpos);
		fprintf(pFile, "PHI_limneg\t%lf\n",PHI_limneg);
		fprintf(pFile, "OX_start\t%lf\n", OX_start);
		fprintf(pFile, "OX_limpos\t%lf\n",OX_limpos);
		fprintf(pFile, "OX_limneg\t%lf\n",OX_limneg);
		fprintf(pFile, "OY_start\t%lf\n", OY_start);
		fprintf(pFile, "OY_limpos\t%lf\n",OY_limpos);
		fprintf(pFile, "OY_limneg\t%lf\n",OY_limneg);
		fprintf(pFile, "OZ_start\t%lf\n", OZ_start);
		fprintf(pFile, "OZ_limpos\t%lf\n",OZ_limpos);
		fprintf(pFile, "OZ_limneg\t%lf\n",OZ_limneg);
		fprintf(pFile, "\n");

		fprintf(pFile, "BX_start\t%lf\n", BX_start);
		fprintf(pFile, "BX_limpos\t%lf\n",BX_limpos);
		fprintf(pFile, "BX_limneg\t%lf\n",BX_limneg);
		fprintf(pFile, "BY_start\t%lf\n", BY_start);
		fprintf(pFile, "BY_limpos\t%lf\n",BY_limpos);
		fprintf(pFile, "BY_limneg\t%lf\n",BY_limneg);
		fprintf(pFile, "BOMEGA_start\t%lf\n", BOMEGA_start);
		fprintf(pFile, "BOMEGA_limpos\t%lf\n",BOMEGA_limpos);
		fprintf(pFile, "BOMEGA_limneg\t%lf\n",BOMEGA_limneg);
		fprintf(pFile, "BCHI_start\t%lf\n", BCHI_start);
		fprintf(pFile, "BCHI_limpos\t%lf\n",BCHI_limpos);
		fprintf(pFile, "BCHI_limneg\t%lf\n",BCHI_limneg);
		fprintf(pFile, "BPHI_start\t%lf\n", BPHI_start);
		fprintf(pFile, "BPHI_limpos\t%lf\n",BPHI_limpos);
		fprintf(pFile, "BPHI_limneg\t%lf\n",BPHI_limneg);
		fclose(pFile);

		printf("File: %s written.\n",filename);
	}
	return 0;
}

int smargon::printParams()
{
		//start writing line by line:	
		printf("l01\t%lf\n",l01);
		printf("l11\t%lf\n",l11);
		printf("l12\t%lf\n",l12);
		printf("l21\t%lf\n",l21);
		printf("l22\t%lf\n",l22);
		printf("l23\t%lf\n",l23);
		printf("l31\t%lf\n",l31);
		printf("l32\t%lf\n",l32);
		printf("l33\t%lf\n",l33);
		printf("l34\t%lf\n",l34);
		printf("l41\t%lf\n",l41);
		printf("l42\t%lf\n",l42);
		printf("l51\t%lf\n",l51);
		printf("l52\t%lf\n",l52);
		printf("l61\t%lf\n",l61);
		printf("l71\t%lf\n",l71);
		printf("l72\t%lf\n",l72);
		printf("l73\t%lf\n",l73);
		printf("l74\t%lf\n",l74);
		printf("offsetQ1\t%lf\n",offsetQ1);
		printf("offsetQ2\t%lf\n",offsetQ2);
		printf("offsetQ3\t%lf\n",offsetQ3);
		printf("offsetQ4\t%lf\n",offsetQ4);
		printf("offsetQ5\t%lf\n",offsetQ5);
		printf("offsetQ6\t%lf\n",offsetQ6);

		printf("q1_start\t%lf\n", q1_start);
		printf("q1_limpos\t%lf\n",q1_limpos);
		printf("q1_limneg\t%lf\n",q1_limneg);
		printf("q2_start\t%lf\n", q2_start);
		printf("q2_limpos\t%lf\n",q2_limpos);
		printf("q2_limneg\t%lf\n",q2_limneg);
		printf("q3_start\t%lf\n", q3_start);
		printf("q3_limpos\t%lf\n",q3_limpos);
		printf("q3_limneg\t%lf\n",q3_limneg);	
		printf("q4_start\t%lf\n", q4_start);
		printf("q4_limpos\t%lf\n",q4_limpos);
		printf("q4_limneg\t%lf\n",q4_limneg);
		printf("q5_start\t%lf\n", q5_start);
		printf("q5_limpos\t%lf\n",q5_limpos);
		printf("q5_limneg\t%lf\n",q5_limneg);
		printf("q6_start\t%lf\n", q6_start);
		printf("q6_limpos\t%lf\n",q6_limpos);
		printf("q6_limneg\t%lf\n",q6_limneg);
		
		printf("SHX_start\t%lf\n", SHX_start);
		printf("SHX_limpos\t%lf\n",SHX_limpos);
		printf("SHX_limneg\t%lf\n",SHX_limneg);
		printf("SHY_start\t%lf\n", SHY_start);
		printf("SHY_limpos\t%lf\n",SHY_limpos);
		printf("SHY_limneg\t%lf\n",SHY_limneg);
		printf("SHZ_start\t%lf\n", SHZ_start);
		printf("SHZ_limpos\t%lf\n",SHZ_limpos);
		printf("SHZ_limneg\t%lf\n",SHZ_limneg);
		printf("OMEGA_start\t%lf\n", OMEGA_start);
		printf("OMEGA_limpos\t%lf\n",OMEGA_limpos);
		printf("OMEGA_limneg\t%lf\n",OMEGA_limneg);
		printf("CHI_start\t%lf\n", CHI_start);
		printf("CHI_limpos\t%lf\n",CHI_limpos);
		printf("CHI_limneg\t%lf\n",CHI_limneg);
		printf("PHI_start\t%lf\n", PHI_start);
		printf("PHI_limpos\t%lf\n",PHI_limpos);
		printf("PHI_limneg\t%lf\n",PHI_limneg);
		printf("OX_start\t%lf\n", OX_start);
		printf("OX_limpos\t%lf\n",OX_limpos);
		printf("OX_limneg\t%lf\n",OX_limneg);
		printf("OY_start\t%lf\n", OY_start);
		printf("OY_limpos\t%lf\n",OY_limpos);
		printf("OY_limneg\t%lf\n",OY_limneg);
		printf("OZ_start\t%lf\n", OZ_start);
		printf("OZ_limpos\t%lf\n",OZ_limpos);
		printf("OZ_limneg\t%lf\n",OZ_limneg);

		printf("BX_start\t%lf\n", BX_start);
		printf("BX_limpos\t%lf\n",BX_limpos);
		printf("BX_limneg\t%lf\n",BX_limneg);
		printf("BY_start\t%lf\n", BY_start);
		printf("BY_limpos\t%lf\n",BY_limpos);
		printf("BY_limneg\t%lf\n",BY_limneg);
		printf("BOMEGA_start\t%lf\n", BOMEGA_start);
		printf("BOMEGA_limpos\t%lf\n",BOMEGA_limpos);
		printf("BOMEGA_limneg\t%lf\n",BOMEGA_limneg);
		printf("BCHI_start\t%lf\n", BCHI_start);
		printf("BCHI_limpos\t%lf\n",BCHI_limpos);
		printf("BCHI_limneg\t%lf\n",BCHI_limneg);
		printf("BPHI_start\t%lf\n", BPHI_start);
		printf("BPHI_limpos\t%lf\n",BPHI_limpos);
		printf("BPHI_limneg\t%lf\n",BPHI_limneg);
	return 0;
}

int smargon::switchOn_CATIALawFile(char* filename) 
{
		char mode[4]="w"; //mode write
		//open file:
		CATIALawFile = fopen(filename, mode);
		//check if file can be opened:
		if (CATIALawFile != NULL){
			//Print Header:
			fprintf(CATIALawFile, "// Prigo II CATIA Law for DMU Kinematics\n");
			fprintf(CATIALawFile, "// Controls Befehl.1,Befehl.2,Befehl.3,Befehl.4 \n");
			fprintf(CATIALawFile, "// Automatically Generated by prigoIK Calculator\n");
			fprintf(CATIALawFile, "// by Wayne Glettig, Oct 2008\n");
			fprintf(CATIALawFile, "// ---------------------------------------------\n");
			fprintf(CATIALawFile, "*COLUMNS = *TIME, Befehl.1, Befehl.2, Befehl.3, Befehl.4\n");
			fprintf(CATIALawFile, "*INTERPOLATION=polyline, polyline, polyline, polyline\n"); //can be polyline or spline
			fprintf(CATIALawFile, "*UNIT=mm, mm, mm, mm\n"); //can be mm, cm, m, deg, rad
			fprintf(CATIALawFile, "*YSCALE=1,1,1,1\n");
			fprintf(CATIALawFile, "\n");
			//Set t counter to 0;
			CATIALawFile_counter =0;
			return 0;
		} else return 1;
}

int smargon::switchOff_CATIALawFile(void) 
{	
		return fclose(CATIALawFile);
}

int smargon::CATIALawFile_writeLine(double UCS1,double UCS2,double UCS3,double UCS4,double UCS5,
		                    double UCS6,double UCS7,double UCS8,double UCS9)
{
	//check if file is open
	if (CATIALawFile != NULL) {
		double UCS[10] = {0.,UCS1,UCS2,UCS3,UCS4,UCS5,UCS6,UCS7,UCS8,UCS9};
		double MCS[10] = {0., 0.,0.,0., 0.,0.,0., 0.,0.,0.};
		int res;
		char errMsg[100], comment[100];
		strcpy (comment,"");
		res = IK(UCS,MCS,errMsg); //run IK model to get Motor Coords
		if (res != 0){  //if an error was encountered, print behind the line //error:...
			strcpy (comment,"//");
			strcat(comment,errMsg);
		}
        	fprintf(CATIALawFile, "%d\t%.4lf\t%.4lf\t%.4lf\t%.4lf %s\n",
				CATIALawFile_counter,
				MCS[1],MCS[2],MCS[3],MCS[4],
				comment);
		CATIALawFile_counter = CATIALawFile_counter + 10;
		return 0;
	} else {
		return 1;
	}

}

int smargon::logString(char* string)
{
	FILE *filehandle;
	char filename[100] = "SmarGon_log.txt", mode[4]="a";
	filehandle = fopen(filename, mode);
	fputs(string,filehandle);//write to file
	fclose(filehandle);
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////
int smargon::IK(double *SCS, double *MCS)
{
	char msg[1000];
	return IK(SCS, MCS, msg);
}
int smargon::IK(double *SCS, double *MCS, char *msg)
{
	double SCSv[9] = {0,0,0, 0,0,0, 0,0,0};
	double MCSv[9] = {0,0,0, 0,0,0, 0,0,0};
        return IK(SCS, SCSv, MCS, MCSv, msg);
}	
int smargon::IK(double *SCS, double *SCSv, double *MCS, double *MCSv, char *msg)
{
	
	//Assign Operational Coordinates
	double SHX = SCS[1]; //sample holder vector
	double SHY = SCS[2];
	double SHZ = SCS[3];
	//double OMEGA = SCS[4]; //UPPER CASE angles are in DEG (lower case are in rad)
	double CHI = SCS[5]; 
	double PHI = SCS[6];
	double OX  = SCS[7];
	double OY  = SCS[8]; 
	double OZ  = SCS[9];
	
	double SHXv = SCSv[1]; //sample holder vector
	double SHYv = SCSv[2];
	double SHZv = SCSv[3];
	//double OMEGAv = SCSv[4]; //UPPER CASE angles are in DEG (lower case are in rad)
	double CHIv = SCSv[5]; 
	double PHIv = SCSv[6];
	double OXv  = SCSv[7];
	double OYv  = SCSv[8]; 
	double OZv  = SCSv[9];


	//convert to radians (note: angles in lowercase are used later on)
	double chi = CHI/360*2*M_PI;
	//double omega = OMEGA/360*2*M_PI;
	double phi =  PHI/360*2*M_PI;
	
	double chiv = CHIv/360*2*M_PI;
	//double omegav = OMEGAv/360*2*M_PI;
	double phiv =  PHIv/360*2*M_PI;


	double q1,q2,q3,q4,q5,q6;
	double q1v,q2v,q3v,q4v,q5v,q6v;

	//IK model :for q1,q2,q3,q5,q6
	q1 = OX - l31 + l42 - (SHZ+l74)*sin(chi) + l71*cos(chi) - SHX*cos(chi)*cos(phi) + SHY*cos(chi)*sin(phi); 
	q1v = OXv - (SHZv*sin(chi) + (SHZ+l74)*cos(chi)*chiv) - l71*sin(chi)*chiv
	       - (SHXv*cos(chi)*cos(phi)-SHX*(sin(chi)*cos(phi)*chiv+cos(chi)*sin(phi)*phiv))
	       + SHYv*cos(chi)*sin(phi)+SHY*(-sin(chi)*chiv*sin(phi)+cos(chi)*cos(phi)*phiv);
	q2 = OY - SHY*cos(phi) - SHX*sin(phi);
	q2v= OYv - (SHYv*cos(phi)-SHY*sin(phi)*phiv) - (SHXv*sin(phi)+SHX*cos(phi)*phiv);
	q3 = OZ - l01 - l23 - l32 - l41 - (SHZ+l74)*cos(chi) - l71*sin(chi) + SHX*cos(phi)*sin(chi) - SHY*sin(chi)*sin(phi);
	q3v= OZv - (SHZv*cos(chi)-(SHZ+l74)*sin(chi)*chiv) - l71*cos(chi)*chiv 
		+ (SHXv*cos(phi)*sin(chi)+SHX*(-sin(phi)*phiv*sin(chi)+cos(phi)*cos(chi)*chiv))
	        - (SHYv*sin(chi)*sin(phi)+SHY*(cos(chi)*chiv*sin(phi)+sin(chi)*cos(phi)*phiv));	
        //q4 needs to be calculated below.
	q5=PHI;    //directly pass on the DEG values 
	q5v=PHIv;
	q6=OMEGA;  
	q6v=OMEGAv;

	// Calculate q4 based on q3 and theta
	//set theta to chi (in rad)
	double theta = chi;
	
	// Use the newton method:
	//set initial q4 value to 0
	q4 = 0;
	// Set loop conditions
	bool loopcond = true;
	int loopcounter = 0;
	int maxloops = 30;
	while (loopcond) {
		double f, f_diff;
		// Newton Formula, using f, and f_diff, 
		f = sqrt(
		        (l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
		       *(l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
	            +   (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
		       *(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
		    ) - l61;
		f_diff = -(2*l32 - 2*l34 + 2*l41 - 2*l51 + 2*q3 - 2*q4 + 2*sin(theta)*(l71 + l73) 
	       		 - 2*l72*cos(theta))/ 
			(2*sqrt(
			    (l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
			   *(l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
		 	 +  (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
			   *(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
			   ));
 		q4 = q4 - (f/f_diff);
		// Increase loop counter, and stop looping if maxloop is reached
    		loopcounter ++;
	    	if (loopcounter> maxloops){
			loopcond = false;
			//ERROR no solution found!!!!
		}
	    	// Calculate residual error, and if sufficiently small stop looping
    		if (fabs(f/f_diff)<1e-9) {
			loopcond = 0;
		}
	}

	// Calculate q4v based on q3v and thetav
	//set theta to chi (in rad)
	double thetav = chiv;
	double dt;
	double q41, q42;
	// Use the newton method:
	//set initial q4 value to 0
	q41 = 0;
	dt=-0.001;
	// Set loop conditions
	loopcond = true;
	loopcounter = 0;
	maxloops = 30;
	while (loopcond) {
		double f, f_diff;
		// Newton Formula, using f, and f_diff, 
		f = sqrt(
		        (l32 - l34 + l41 - l51 + q3v*dt - q41 + sin(thetav*dt)*(l71 + l73) - l72*cos(thetav*dt))
		       *(l32 - l34 + l41 - l51 + q3v*dt - q41 + sin(thetav*dt)*(l71 + l73) - l72*cos(thetav*dt))
	            +   (l31 + l33 - l42 + l52 - cos(thetav*dt)*(l71 + l73) - l72*sin(thetav*dt))
		       *(l31 + l33 - l42 + l52 - cos(thetav*dt)*(l71 + l73) - l72*sin(thetav*dt))
		    ) - l61;
		f_diff = -(2*l32 - 2*l34 + 2*l41 - 2*l51 + 2*q3v*dt - 2*q41 + 2*sin(thetav*dt)*(l71 + l73) 
	       		 - 2*l72*cos(thetav*dt))/ 
			(2*sqrt(
			    (l32 - l34 + l41 - l51 + q3v*dt - q41 + sin(thetav*dt)*(l71 + l73) - l72*cos(thetav*dt))
			   *(l32 - l34 + l41 - l51 + q3v*dt - q41 + sin(thetav*dt)*(l71 + l73) - l72*cos(thetav*dt))
		 	 +  (l31 + l33 - l42 + l52 - cos(thetav*dt)*(l71 + l73) - l72*sin(thetav*dt))
			   *(l31 + l33 - l42 + l52 - cos(thetav*dt)*(l71 + l73) - l72*sin(thetav*dt))
			   ));
 		q41 = q41 - (f/f_diff);
		// Increase loop counter, and stop looping if maxloop is reached
    		loopcounter ++;
	    	if (loopcounter> maxloops){
			loopcond = false;
			//ERROR no solution found!!!!
		}
	    	// Calculate residual error, and if sufficiently small stop looping
    		if (fabs(f/f_diff)<1e-9) {
			loopcond = 0;
		}
	}

	// Use the newton method:
	//set initial q4 value to 0
	q42 = 0;
	dt = 0.001;
	// Set loop conditions
	loopcond = true;
	loopcounter = 0;
	maxloops = 30;
	while (loopcond) {
		double f, f_diff;
		// Newton Formula, using f, and f_diff, 
		f = sqrt(
		        (l32 - l34 + l41 - l51 + q3v*dt - q42 + sin(thetav*dt)*(l71 + l73) - l72*cos(thetav*dt))
		       *(l32 - l34 + l41 - l51 + q3v*dt - q42 + sin(thetav*dt)*(l71 + l73) - l72*cos(thetav*dt))
	            +   (l31 + l33 - l42 + l52 - cos(thetav*dt)*(l71 + l73) - l72*sin(thetav*dt))
		       *(l31 + l33 - l42 + l52 - cos(thetav*dt)*(l71 + l73) - l72*sin(thetav*dt))
		    ) - l61;
		f_diff = -(2*l32 - 2*l34 + 2*l41 - 2*l51 + 2*q3v*dt - 2*q42 + 2*sin(thetav*dt)*(l71 + l73) 
	       		 - 2*l72*cos(thetav*dt))/ 
			(2*sqrt(
			    (l32 - l34 + l41 - l51 + q3v*dt - q42 + sin(thetav*dt)*(l71 + l73) - l72*cos(thetav*dt))
			   *(l32 - l34 + l41 - l51 + q3v*dt - q42 + sin(thetav*dt)*(l71 + l73) - l72*cos(thetav*dt))
		 	 +  (l31 + l33 - l42 + l52 - cos(thetav*dt)*(l71 + l73) - l72*sin(thetav*dt))
			   *(l31 + l33 - l42 + l52 - cos(thetav*dt)*(l71 + l73) - l72*sin(thetav*dt))
			   ));
 		q42 = q42 - (f/f_diff);
		// Increase loop counter, and stop looping if maxloop is reached
    		loopcounter ++;
	    	if (loopcounter> maxloops){
			loopcond = false;
			//ERROR no solution found!!!!
		}
	    	// Calculate residual error, and if sufficiently small stop looping
    		if (fabs(f/f_diff)<1e-9) {
			loopcond = 0;
		}
	}

	q4v=(q42-q41)/(2*dt);

	//export parameters	
	MCS[1] = q1 - offsetQ1;
	MCS[2] = q2 - offsetQ2;
	MCS[3] = q3 - offsetQ3;
	MCS[4] = q4 - offsetQ4;
	MCS[5] = q5 - offsetQ5;
	MCS[6] = q6 - offsetQ6;
	MCS[7] = SHX;
	MCS[8] = SHY;
	MCS[9] = SHZ;
	
	MCSv[1] = q1v;
	MCSv[2] = q2v;
	MCSv[3] = q3v;
	MCSv[4] = q4v;
	MCSv[5] = q5v;
	MCSv[6] = q6v;
	
	sprintf(msg, "ok");
	return 0;
}



int smargon::FK(double *MCS, double *SCS, char *msg)
{

	//Assign Articular Coordinates
	double q1, q2, q3, q4, q5, q6; 
	q1 = MCS[1] + offsetQ1; //Slider positions
	q2 = MCS[2] + offsetQ2;
	q3 = MCS[3] + offsetQ3;
	q4 = MCS[4] + offsetQ4; 
	q5 = MCS[5] + offsetQ5; 
	q6 = MCS[6] + offsetQ6; 

		
	//double SHY  = MCS[7]; //Centre of Interest (described with Frame 0)
	//double SHZ  = MCS[8]; 
	//double SHX  = MCS[9];

	
	// Calculate theta (based on q3 & q4)
	// Use the newton method:
	// Starting value theta_0:
	double theta = 0;
	// Set loop conditions
	bool loopcond = true;
	int loopcounter = 0;
	int maxloops = 30;
	while (loopcond) {
		double f, f_diff;
		// Newton Formula, using f, and f_diff, 
		f = sqrt(
		        (l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
		       *(l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
	            +   (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
		       *(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
		    ) - l61;
		f_diff = (2*(cos(theta)*(l71 + l73) + l72*sin(theta))
		           *(l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta)) 
			   + 2*(sin(theta)*(l71 + l73) - l72*cos(theta))
			   *(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta)))
			/(2*sqrt(
		            (l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
		           *(l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
		 	 +  (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
			   *(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
			   ));

 		theta = theta - (f/f_diff);
		// Increase loop counter, and stop looping if maxloop is reached
    		loopcounter ++;
	    	if (loopcounter> maxloops){
			loopcond = false;
			//ERROR no solution found!!!!
		}
	    	// Calculate residual error, and if sufficiently small stop looping
    		if (fabs(f/f_diff)<1e-9) {
			loopcond = 0;
		}
	}


	// The FK Model is:
	double X, Y, Z, /*OMEGA,*/ CHI, PHI, phi, omega;
	phi = q5/180*M_PI;
	omega = q6/180*M_PI;
	X = l31*cos(omega) - SHY*(cos(phi)*sin(omega) + cos(omega)*cos(theta)*sin(phi)) 
	  - SHX*(sin(phi)*sin(omega) - cos(phi)*cos(omega)*cos(theta)) 
	  - l42*cos(omega) + q1*cos(omega) - q2*sin(omega) + SHZ*cos(omega)*sin(theta) 
	  - l71*cos(omega)*cos(theta) + l74*cos(omega)*sin(theta);
	Y = SHX*(cos(omega)*sin(phi) + cos(phi)*cos(theta)*sin(omega)) 
	  + SHY*(cos(phi)*cos(omega) - cos(theta)*sin(phi)*sin(omega)) 
	  + q2*cos(omega) + l31*sin(omega) - l42*sin(omega) 
	  + q1*sin(omega) + SHZ*sin(omega)*sin(theta) 
	  - l71*cos(theta)*sin(omega) + l74*sin(omega)*sin(theta);
	Z = l01 + l23 + l32 + l41 + q3 + SHZ*cos(theta) + l74*cos(theta) 
	  + l71*sin(theta) - SHX*cos(phi)*sin(theta) + SHY*sin(phi)*sin(theta);
	//OMEGA = q6;
	CHI = (180*theta)/M_PI;
	PHI = q5;

	//export parameters
	SCS[1] = SHX;
	SCS[2] = SHY;
	SCS[3] = SHZ;
	SCS[4] = OMEGA;
	SCS[5] = CHI;
	SCS[6] = PHI;
	SCS[7] = X;
	SCS[8] = Y;
	SCS[9] = Z;

	sprintf(msg, "ok");
	
	return 0;
}

//overloaded mgi function without msg feedback
int smargon::FK(double *MCS, double *SCS)
{
	char msg[1000];
	return FK(MCS, SCS, msg);
}


/*int smargon::BCStoMCS(double *BCS, double *SCS)
{
	double BLX = OX+BCS[1];
	double BLY = OY+BCS[2];
	double BLZ = OZ;
        //also uses CHI, q1-q6
	SCS[1] = SHX;
	SCS[2] = SHY;
	SCS[3] = SHZ;

        return 0;
} */

int smargon::runIK(void)
{
	double SCS[10];
	double MCS[10];
	char msg[1000];
        SCS[1] = SHX;
        SCS[2] = SHY;
        SCS[3] = SHZ;
        SCS[4] = OMEGA;
        SCS[5] = CHI;
        SCS[6] = PHI;
        SCS[7] = OX;
        SCS[8] = OY;
        SCS[9] = OZ;

	IK(SCS,MCS,msg);

	q1  = MCS[1];
	q2  = MCS[2];
	q3  = MCS[3];
	q4  = MCS[4]; 
	q5  = MCS[5]; 
	q6  = MCS[6]; 
	SHX  = MCS[7];
	SHY  = MCS[8]; 
	SHZ  = MCS[9];
	return 0;
}

int smargon::runIKv(void)
{
	double SCS[10];
	double SCSv[10];
	double MCS[10];
	double MCSv[10];
	char msg[1000];
        SCS[1] = SHX;   SCSv[1] = SHXv;
        SCS[2] = SHY;   SCSv[2] = SHYv;
        SCS[3] = SHZ;   SCSv[3] = SHZv;
        SCS[4] = OMEGA; SCSv[4] = OMEGAv;
        SCS[5] = CHI;   SCSv[5] = CHIv;
        SCS[6] = PHI;   SCSv[6] = PHIv;
        SCS[7] = OX;    SCSv[7] = OXv;
        SCS[8] = OY;    SCSv[8] = OYv;
        SCS[9] = OZ;    SCSv[9] = OZv;

	IK(SCS,SCSv,MCS,MCSv,msg);

	q1  = MCS[1];   q1v  = MCSv[1]; 
	q2  = MCS[2];   q2v  = MCSv[2];
	q3  = MCS[3];   q3v  = MCSv[3];
	q4  = MCS[4];   q4v  = MCSv[4]; 
	q5  = MCS[5];   q5v  = MCSv[5]; 
	q6  = MCS[6];   q6v  = MCSv[6]; 
	SHX  = MCS[7];  
	SHY  = MCS[8];  
	SHZ  = MCS[9];  
	return 0;
}
int smargon::runFK(void)
{
	double SCS[10];
	double MCS[10];
	char msg[1000];
	//q1  = MCS[1] = q1;
	//q2  = MCS[2] = q2;
	//q3  = MCS[3] = q3;
	//q4  = MCS[4] = q4; 
	//q5  = MCS[5] = q5; 
	//q6  = MCS[6] = q6; 
	MCS[1] = q1;
	MCS[2] = q2;
	MCS[3] = q3;
	MCS[4] = q4; 
	MCS[5] = q5; 
	MCS[6] = q6; 

	FK(MCS,SCS,msg);

        SHX = SCS[1];
        SHY = SCS[2];
        SHZ = SCS[3];
        //OMEGA = SCS[4];
        CHI = SCS[5];
        PHI = SCS[6];
        OX = SCS[7];
        OY = SCS[8];
        OZ = SCS[9];
	return 0;
}




int smargon::traj_reset()
{
        for (int i=0; i<5; i++) {
                traj_x[i] = traj_xT[i];
                traj_v[i] = 0;
                traj_a[i] = 0;
        }
	return 0;
}



int smargon::traj_runstep()
{
    for (int i=0; i<5; i++){
	if (traj_v[i] == 0) {
	    traj_a[i] = sgn(traj_xT[i]-traj_x[i])*traj_amax[i];
	}
	else {
	    double xmax = traj_x[i] + 1/2*traj_v[i]*traj_v[i]/sgn(traj_v[i])/traj_amax[i];
	    if (xmax > traj_xT[i]) traj_a[i]=-traj_amax[i];
	    else traj_a[i]=traj_amax[i];
	}
	traj_v[i] += traj_a[i]*dt;
	if (traj_v[i]>traj_vmax[i]){
	    traj_v[i]=traj_vmax[i];
	    traj_a[i]=0;
	}
	if (traj_v[i]<-traj_vmax[i]){
	    traj_v[i]=-traj_vmax[i];
	    traj_a[i]=0;
	}

        traj_x[i] += traj_v[i]*dt + 1/2*traj_a[i]*dt*dt;
        
       	//if we hit xT before the end of the next dt, keep v and set x to xT 
	if (fabs(traj_xT[i]-traj_x[i])<0.001 && fabs(traj_v[i])<0.1) {
            traj_x[i]=traj_xT[i];
	}
    }
    return 0;
}
