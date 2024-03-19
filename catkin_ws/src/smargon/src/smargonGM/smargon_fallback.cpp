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



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
smargon::smargon()
{
    //initialize lengths and distances:
    //all lengths in [mm]
    l01 = 42.5;
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
    l52 =  2.5;
    l61 = 64.422; //Connecting rod length
    l71 = 53;  //Swing dimensions
    l72 = 17.67;
    l73 = 5.2;
    l74 = 1.53;
    
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
    OZ = 275;

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
			if      (strcmp(read_str, "SHX")==0){ SHX=read_double;printf("SHX=%lf\n",read_double);}
			else if (strcmp(read_str, "SHY")==0){ SHY=read_double;printf("SHY=%lf\n",read_double);}
			else if (strcmp(read_str, "SHZ")==0){ SHZ=read_double;printf("SHZ=%lf\n",read_double);}
			else if (strcmp(read_str, "OX")==0){ OX=read_double;printf("OX=%lf\n",read_double);}
			else if (strcmp(read_str, "OY")==0){ OY=read_double;printf("OY=%lf\n",read_double);}
			else if (strcmp(read_str, "OZ")==0){ OZ=read_double;printf("OZ=%lf\n",read_double);}
			
			else if (strcmp(read_str, "offsetQ1")==0){ offsetQ1=read_double;printf("offsetQ1=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ2")==0){ offsetQ2=read_double;printf("offsetQ2=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ3")==0){ offsetQ3=read_double;printf("offsetQ3=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ4")==0){ offsetQ4=read_double;printf("offsetQ4=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ5")==0){ offsetQ5=read_double;printf("offsetQ5=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetQ6")==0){ offsetQ6=read_double;printf("offsetQ6=%lf\n",read_double);}

			else if (strcmp(read_str, "l01")==0){ l01=read_double;printf("l01=%lf\n",read_double);}

			else if (strcmp(read_str, "l11")==0){ l11=read_double;printf("l11=%lf\n",read_double);}
			else if (strcmp(read_str, "l12")==0){ l12=read_double;printf("l12=%lf\n",read_double);}

			else if (strcmp(read_str, "l21")==0){ l21=read_double;printf("l21=%lf\n",read_double);}
			else if (strcmp(read_str, "l22")==0){ l22=read_double;printf("l22=%lf\n",read_double);}
			else if (strcmp(read_str, "l23")==0){ l23=read_double;printf("l23=%lf\n",read_double);}

			else if (strcmp(read_str, "l31")==0){ l31=read_double;printf("l31=%lf\n",read_double);}
			else if (strcmp(read_str, "l32")==0){ l32=read_double;printf("l32=%lf\n",read_double);}
			else if (strcmp(read_str, "l33")==0){ l33=read_double;printf("l33=%lf\n",read_double);}
			else if (strcmp(read_str, "l34")==0){ l34=read_double;printf("l34=%lf\n",read_double);}

			else if (strcmp(read_str, "l41")==0){ l41=read_double;printf("l41=%lf\n",read_double);}
			else if (strcmp(read_str, "l42")==0){ l42=read_double;printf("l42=%lf\n",read_double);}

			else if (strcmp(read_str, "l51")==0){ l51=read_double;printf("l51=%lf\n",read_double);}
			else if (strcmp(read_str, "l52")==0){ l52=read_double;printf("l52=%lf\n",read_double);}
			else if (strcmp(read_str, "l61")==0){ l61=read_double;printf("l61=%lf\n",read_double);}
	
			else if (strcmp(read_str, "l71")==0){ l71=read_double;printf("l71=%lf\n",read_double);}
			else if (strcmp(read_str, "l72")==0){ l72=read_double;printf("l72=%lf\n",read_double);}
			else if (strcmp(read_str, "l73")==0){ l73=read_double;printf("l73=%lf\n",read_double);}
			else if (strcmp(read_str, "l74")==0){ l74=read_double;printf("l74=%lf\n",read_double);}
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
	} else {
		//start writing line by line:
		fprintf(pFile, "SHX\t%lf\n",SHX);
		fprintf(pFile, "SHY\t%lf\n",SHY);
		fprintf(pFile, "SHZ\t%lf\n",SHZ);
		fprintf(pFile, "OX\t%lf\n",OX);
		fprintf(pFile, "OY\t%lf\n",OY);
		fprintf(pFile, "OZ\t%lf\n",OZ);

		fprintf(pFile, "offsetQ1\t%lf\n",offsetQ1);
		fprintf(pFile, "offsetQ2\t%lf\n",offsetQ2);
		fprintf(pFile, "offsetQ3\t%lf\n",offsetQ3);
		fprintf(pFile, "offsetQ4\t%lf\n",offsetQ4);
		fprintf(pFile, "offsetQ5\t%lf\n",offsetQ5);
		fprintf(pFile, "offsetQ6\t%lf\n",offsetQ6);

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

		fclose(pFile);
	}
	return 0;
}

int smargon::printParams()
{
		//start writing line by line:
		printf("SHX\t%lf\n",SHX);
		printf("SHY\t%lf\n",SHY);
		printf("SHZ\t%lf\n",SHZ);
		printf("OX\t%lf\n",OX);
		printf("OY\t%lf\n",OY);
		printf("OZ\t%lf\n",OZ);

		printf("offsetQ1\t%lf\n",offsetQ1);
		printf("offsetQ2\t%lf\n",offsetQ2);
		printf("offsetQ3\t%lf\n",offsetQ3);
		printf("offsetQ4\t%lf\n",offsetQ4);
		printf("offsetQ5\t%lf\n",offsetQ5);
		printf("offsetQ6\t%lf\n",offsetQ6);

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
int smargon::IK(double *SCS, double *MCS, char *msg)
{
	//Assign Operational Coordinates
	double SHY = SCS[1]; //sample holder vector
	double SHZ = SCS[2];
	double SHX = SCS[3];
	double CHI = SCS[4]; //UPPER CASE angles are in DEG (lower case are in rad)
	double PHI = SCS[5]; 
	double OMEGA = SCS[6];

	double Y = SCS[7];
	double Z = SCS[8]; 
	double X = SCS[9];


	//convert to radians (note: angles in lowercase are used later on)
	double chi = CHI/360*2*Pi;
	double omega = OMEGA/360*2*Pi;
	double phi =  PHI/360*2*Pi;
	//set initial value for theta
	double theta = chi;

	//Converge towards a solution, q are motor coordinates, x are user coordinates
	
	//Set Target User Coords (X, Y, Z, OMEGA, CHI, PHI
	double xt[6]={X,Y,Z,OMEGA,CHI,PHI};

	//Set Motor Start Values (q1,q2,q3, theta, q5,q6)
	double qc[6]={Z,Y,X,CHI,PHI,OMEGA};
	bool loopcond = true;
	int loopcounter = 0;
	int maxloops=30;
	while (loopcond) {
		//get current x values based on q 
		double xc[6];
		xc[0]= l01 + l23 + l32 + l41 + qc[2] + SHX*cos(qc[3]) + l74*cos(qc[3]) 
			+ l71*sin(qc[3]) - SHY*cos(qc[4])*sin(qc[3]) + SHZ*sin(qc[4])*sin(qc[3]);
		xc[1]= l31*cos(qc[5]) - SHZ*(cos(qc[4])*sin(qc[5]) + cos(qc[5])*cos(qc[3])*sin(qc[4])) 
			- SHY*(sin(qc[4])*sin(qc[5]) - cos(qc[4])*cos(qc[5])*cos(qc[3])) 
			- l42*cos(qc[5]) + qc[0]*cos(qc[5]) - qc[1]*sin(qc[5]) 
			+ SHX*cos(qc[5])*sin(qc[3]) - l71*cos(qc[5])*cos(qc[3]) 
			+ l74*cos(qc[5])*sin(qc[3]);
		xc[2]= SHY*(cos(qc[5])*sin(qc[4]) + cos(qc[4])*cos(qc[3])*sin(qc[5])) 
			+ SHZ*(cos(qc[4])*cos(qc[5]) - cos(qc[3])*sin(qc[4])*sin(qc[5])) 
			+ qc[1]*cos(qc[5]) + l31*sin(qc[5]) - l42*sin(qc[5]) + qc[0]*sin(qc[5]) 
			+ SHX*sin(qc[5])*sin(qc[3]) - l71*cos(qc[3])*sin(qc[5]) 
			+ l74*sin(qc[5])*sin(qc[3]);
		xc[3]= (180*qc[5])/M_PI;
		xc[4]= (180*qc[3])/M_PI;
		xc[5]= (180*qc[4])/M_PI;

//                printf("xc: %lf %lf %lf %lf %lf %lf \n", xc[0],xc[1],xc[2],xc[3],xc[4],xc[5]);

		//get x error (target - current)
		double deltax[6];
		deltax[0]=xt[0]-xc[0]; deltax[1]=xt[1]-xc[1]; deltax[2]=xt[2]-xc[2];
		deltax[3]=xt[3]-xc[3]; deltax[4]=xt[4]-xc[4]; deltax[5]=xt[5]-xc[5];
		double deltax_abs; 
		deltax_abs = sqrt(deltax[0]*deltax[0] + deltax[1]*deltax[1] + deltax[2]*deltax[2] +
				  deltax[3]*deltax[3] + deltax[4]*deltax[4] + deltax[5]*deltax[5]); 

//                printf("deltax: %lf \n", deltax_abs);

		if (deltax_abs < 1e-9) { //If abs error small enough, get out of loop
			loopcond = false;
		}

		//calculate the Jacobian:
		double J[6][6];
		J[0][0] = 0; 
		J[0][1] = 0; 
		J[0][2] = 1; 
		J[0][3] = l71*cos(qc[3]) - SHX*sin(qc[3]) - l74*sin(qc[3]) 
			- SHY*cos(qc[4])*cos(qc[3]) + SHZ*cos(qc[3])*sin(qc[4]); 
		J[0][4] = SHZ*cos(qc[4])*sin(qc[3]) + SHY*sin(qc[4])*sin(qc[3]); 
		J[0][5] = 0; 

		J[1][0] = cos(qc[5]);
		J[1][1] = -sin(qc[5]);
		J[1][2] = 0;
		J[1][3] = SHX*cos(qc[5])*cos(qc[3]) + l74*cos(qc[5])*cos(qc[3]) 
			+ l71*cos(qc[5])*sin(qc[3]) - SHY*cos(qc[4])*cos(qc[5])*sin(qc[3]) 
			+ SHZ*cos(qc[5])*sin(qc[4])*sin(qc[3]);
		J[1][4] = SHZ*(sin(qc[4])*sin(qc[5]) - cos(qc[4])*cos(qc[5])*cos(qc[3])) 
			- SHY*(cos(qc[4])*sin(qc[5]) + cos(qc[5])*cos(qc[3])*sin(qc[4]));
		J[1][5] = l42*sin(qc[5]) - SHZ*(cos(qc[4])*cos(qc[5]) 
			- cos(qc[3])*sin(qc[4])*sin(qc[5])) - qc[1]*cos(qc[5]) 
			- l31*sin(qc[5]) - SHY*(cos(qc[5])*sin(qc[4]) 
			+ cos(qc[4])*cos(qc[3])*sin(qc[5])) - qc[0]*sin(qc[5]) 
			- SHX*sin(qc[5])*sin(qc[3]) + l71*cos(qc[3])*sin(qc[5]) 
			- l74*sin(qc[5])*sin(qc[3]);

		J[2][0] = sin(qc[5]);
		J[2][1] = cos(qc[5]);
		J[2][2] = 0;
		J[2][3] = SHX*cos(qc[3])*sin(qc[5]) + l74*cos(qc[3])*sin(qc[5]) 
			+ l71*sin(qc[5])*sin(qc[3]) - SHY*cos(qc[4])*sin(qc[5])*sin(qc[3]) 
			+ SHZ*sin(qc[4])*sin(qc[5])*sin(qc[3]);
		J[2][4] = SHY*(cos(qc[4])*cos(qc[5]) - cos(qc[3])*sin(qc[4])*sin(qc[5])) 
			- SHZ*(cos(qc[5])*sin(qc[4]) + cos(qc[4])*cos(qc[3])*sin(qc[5]));
		J[2][5] = l31*cos(qc[5]) - SHZ*(cos(qc[4])*sin(qc[5]) 
			+ cos(qc[5])*cos(qc[3])*sin(qc[4])) - SHY*(sin(qc[4])*sin(qc[5]) 
			- cos(qc[4])*cos(qc[5])*cos(qc[3])) - l42*cos(qc[5]) 
			+ qc[0]*cos(qc[5]) - qc[1]*sin(qc[5]) + SHX*cos(qc[5])*sin(qc[3]) 
			- l71*cos(qc[5])*cos(qc[3]) + l74*cos(qc[5])*sin(qc[3]);

		J[3][0] = 0;
		J[3][1] = 0;
		J[3][2] = 0;
		J[3][3] = 0;
		J[3][4] = 0;
		J[3][5] = 180/M_PI;

		J[4][0] = 0;
		J[4][1] = 0;
		J[4][2] = 0;
		J[4][3] = 180/M_PI;
		J[4][4] = 0;
		J[4][5] = 0;

		J[5][0] = 0;
		J[5][1] = 0;
		J[5][2] = 0;
		J[5][3] = 0;
		J[5][4] = 180/M_PI;
		J[5][5] = 0;

	
//                printf("J0: %lf %lf %lf %lf %lf %lf \n", J[0][0],J[0][1],J[0][2],J[0][3],J[0][4],J[0][5]);
//                printf("J1: %lf %lf %lf %lf %lf %lf \n", J[1][0],J[1][1],J[1][2],J[1][3],J[1][4],J[1][5]);
//                printf("J2: %lf %lf %lf %lf %lf %lf \n", J[2][0],J[2][1],J[2][2],J[2][3],J[2][4],J[2][5]);
//                printf("J3: %lf %lf %lf %lf %lf %lf \n", J[3][0],J[3][1],J[3][2],J[3][3],J[3][4],J[3][5]);
//                printf("J4: %lf %lf %lf %lf %lf %lf \n", J[4][0],J[4][1],J[4][2],J[4][3],J[4][4],J[4][5]);
//                printf("J5: %lf %lf %lf %lf %lf %lf \n", J[5][0],J[5][1],J[5][2],J[5][3],J[5][4],J[5][5]);

		//Solve equation system (LU Decomposition, LU Solve)
		int decompOK = 0, solveOK = 0;
		int pivot[6];	
		double deltaq[6];
		decompOK= Doolittle_LU_Decomposition_with_Pivoting(&J[0][0], pivot, 6);
		solveOK = Doolittle_LU_with_Pivoting_Solve(&J[0][0], deltax, pivot, deltaq, 6);
	
//                printf("1:%d, 2:%d, deltaq: %lf %lf %lf %lf %lf %lf \n", decompOK,solveOK,deltaq[0],deltaq[1],deltaq[2],deltaq[3],deltaq[4],deltaq[5]);
		
		//Newton Corde (Xn+1 = Xn - yn)
		qc[0] += deltaq[0];
		qc[1] += deltaq[1];
		qc[2] += deltaq[2];
		qc[3] += deltaq[3];
		qc[4] += deltaq[4];
		qc[5] += deltaq[5];

		//Loop condition: If precision better than a nanometer ((10^-6)^2 = 10^-12 [mm]) exit loop
		if (NormeCarreVect(deltaq,6) < 1e-6)
			loopcond = false;

		//Stop looping if more than 30 iterations (or maxloops)  were done, sound alarm.
		loopcounter++; //Increment Loop Counter
		if (loopcounter > maxloops) {
			loopcond = false;
			sprintf(msg, "IK (6x6 matrix solve) did not converge after %d iterations\n", maxloops);
			return 2;
		}

	}
	// Calculate q4 based on q3 and theta
	// H35 has q4 in it, which must be calculated.

	// Distance between P560in5 and P770in5
	double P560in5[4];
	P560in5[0]=l51; P560in5[1]=-l52, P560in5[2]=0, P560in5[3]=1;
	double P770in7[4];
	P770in7[0]=-l72; P770in7[1]=-l71-l73, P770in7[2]=0, P770in7[3]=1;
	
	// Use the newton method:
	// Starting value theta_0:
	double q4 = 0;
	theta = qc[3];
	// Set loop conditions (variables have been declared above in first loop)
	loopcond = true;
	loopcounter = 0;
	maxloops = 30;
	while (loopcond) {
		double f, f_diff;
		// Newton Formula, using f, and f_diff, 
		f = sqrt(
		        (l32 - l34 + l41 - l51 + qc[2] - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
		       *(l32 - l34 + l41 - l51 + qc[2] - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
	            +   (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
		       *(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
		    ) - l61;
		f_diff = -(2*l32 - 2*l34 + 2*l41 - 2*l51 + 2*qc[2] - 2*q4 + 2*sin(theta)*(l71 + l73) 
	       		 - 2*l72*cos(theta))/ 
			(2*sqrt(
			    (l32 - l34 + l41 - l51 + qc[2] - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
			   *(l32 - l34 + l41 - l51 + qc[2] - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
		 	 +  (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
			   *(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
			   ));
 		q4 = q4 - (f/f_diff);
//	    	printf("f, f_diff, q4, %lf, %lf, %lf\n",f, f_diff,q4);
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

	//export parameters	
	MCS[1] = qc[0];
	MCS[2] = qc[1];
	MCS[3] = qc[2];
	MCS[4] = q4;
	MCS[5] = qc[4];
	MCS[6] = qc[5];
	MCS[7] = SHY;
	MCS[8] = SHZ;
	MCS[9] = SHX;
	
	sprintf(msg, "ok");
	return 0;
}

//overloaded mgi function without msg feedback
int smargon::IK(double *SCS, double *MCS)
{
	char msg[1000];
	return IK(SCS, MCS, msg);
}


int smargon::FK(double *MCS, double *SCS, char *msg)
{

	//Assign Articular Coordinates
	double q1  = MCS[1]; //Slider positions
	double q2  = MCS[2];
	double q3  = MCS[3];
	double q4  = MCS[4]; 
	double q5  = MCS[5]; 
	double q6  = MCS[6]; 

	double SHY  = MCS[7]; //Centre of Interest (described with Frame 0)
	double SHZ  = MCS[8]; 
	double SHX  = MCS[9];

	
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
	double X, Y, Z, OMEGA, CHI, PHI;
	X = l01 + l23 + l32 + l41 + q3 + SHX*cos(theta) + l74*cos(theta) 
	  + l71*sin(theta) - SHY*cos(q5)*sin(theta) + SHZ*sin(q5)*sin(theta);
	Y = l31*cos(q6) - SHZ*(cos(q5)*sin(q6) + cos(q6)*cos(theta)*sin(q5)) 
	  - SHY*(sin(q5)*sin(q6) - cos(q5)*cos(q6)*cos(theta)) 
	  - l42*cos(q6) + q1*cos(q6) - q2*sin(q6) + SHX*cos(q6)*sin(theta) 
	  - l71*cos(q6)*cos(theta) + l74*cos(q6)*sin(theta);
	Z = SHY*(cos(q6)*sin(q5) + cos(q5)*cos(theta)*sin(q6)) 
	  + SHZ*(cos(q5)*cos(q6) - cos(theta)*sin(q5)*sin(q6)) 
	  + q2*cos(q6) + l31*sin(q6) - l42*sin(q6) 
	  + q1*sin(q6) + SHX*sin(q6)*sin(theta) 
	  - l71*cos(theta)*sin(q6) + l74*sin(q6)*sin(theta);
	OMEGA = (180*q6)/M_PI;
	CHI = (180*theta)/M_PI;
	PHI = (180*q5)/M_PI;

	
	//export parameters
	SCS[1] = SHY;
	SCS[2] = SHZ;
	SCS[3] = SHX;
	SCS[4] = CHI;
	SCS[5] = PHI;
	SCS[6] = OMEGA;
	SCS[7] = Y;
	SCS[8] = Z;
	SCS[9] = X;

	sprintf(msg, "ok");
	
	return 0;
}

//overloaded mgi function without msg feedback
int smargon::FK(double *MCS, double *SCS)
{
	char msg[1000];
	return FK(MCS, SCS, msg);
}


int smargon::BCStoMCS(double *BCS, double *SCS)
{
	double BLX = OX+BCS[1];
	double BLY = OY+BCS[2];
	double BLZ = OZ;
        //also uses CHI, q1-q6
	SCS[1] = SHX;
	SCS[2] = SHY;
	SCS[3] = SHZ;


} 

int smargon::runIK(void)
{
	double SCS[10];
	double MCS[10];
	char msg[1000];
        SCS[1] = SHX;
        SCS[2] = SHY;
        SCS[3] = SHZ;
        SCS[4] = CHI;
        SCS[5] = PHI;
        SCS[6] = OMEGA;
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
}
