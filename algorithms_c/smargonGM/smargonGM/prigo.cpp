// prigo.cpp: implementation of the prigo class.
// Modified 18.12.2011 by Wayne Glettig, CSEM
// Modified 29.2.2012 by Wayne Glettig, CSEM
//////////////////////////////////////////////////////////////////////
//#include "stdafx.h" 
#include <stdio.h>
#include <string.h>

#include "math.h"
#include "Global.h"
#include "MatriceVectLib.h"  //LSRO Matrix library (wayne erweitert)
#include "prigo.h"

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

prigo::prigo()
{
	//initialize lengths and distances:
	l11 = 7.;
	l12 = 11.;
	l13 = 16.;
	l14 = 5.;
	
	l24 = 61;

	l31 = 86;
	l34 = 29;
	l35 = 12;

	l41 = 33;
	l42 = 33;
	l43 = 33;
	l44 = 100;

	l51 = 192;
	l52 = 23;
	l53 = 14;
	l54 = 17.63;
	l55 = 22.5008;
	l56 = 2*42.576;

	a5 = 20*2*Pi/360;

	//calculated lengths (Folgelängen)
	l21 = sqrt(l35*l35 + l24*l24);
	l22 = sqrt((l31/2)*(l31/2) + l34*l34 + l24*l24);
	l23 = l22;
	l32 = sqrt((l31/2.)*(l31/2.) + (l34+l35)*(l34+l35));
	l33 = l32;


	//offsets Reference (base)
	//Reference: for straight sliders (1,2,3) z coordinate of 0
	//Referemce: for slider 4: P5 (positive is up)
	offsetR1 = 75 + 93+7.68117-1.36;
	offsetR2 = 75 + 93;
	offsetR3 = 75 + 93-0.378;
	offsetR4 = 130.552 - 85+10.5647-0.28;
    
	//offsets Sliders
	//Sliders: Offset is measured from P1,P2,P3,B5
/*	offsetS1 = 93+0.00+7.68117;//93+0.05+7.89;
	offsetS2 = 93+0.00+0;//93+0.15+0.509;
	offsetS3 = 93+0.00+0.43495;//93+0.15;
	offsetS4 = 85-0.03+10.5647;
*/
	//Fine tuning: offsets should be around max +/-1mm
	offsetS1 = 0;
	offsetS2 = 0;
	offsetS3 = 0;
	offsetS4 = 0;


	SHx = 0;
	SHy = 0;
	SHz = 20;

	Ox = 0;
	Oy = 0;
	Oz = 275;

}

prigo::~prigo()
{

}

void prigo::setOffsetR1(double offset)
{
	offsetR1 = offset;

	return;
}
void prigo::setOffsetR2(double offset)
{
	offsetR2 = offset;
	return;
}
void prigo::setOffsetR3(double offset)
{
	offsetR3 = offset;
	return;
}
void prigo::setOffsetR4(double offset)
{
	offsetR4 = offset;
	return;
}

void prigo::setOffsetS1(double offset)
{
	offsetS1 = offset;
	return;
}
void prigo::setOffsetS2(double offset)
{
	offsetS2 = offset;
	return;
}
void prigo::setOffsetS3(double offset)
{
	offsetS3 = offset;
	return;
}
void prigo::setOffsetS4(double offset)
{
	offsetS4 = offset;
	return;
}
double prigo::getOffsetR1()
{
	return offsetR1;
}
double prigo::getOffsetR2()
{
	return offsetR2;
}
double prigo::getOffsetR3()
{
	return offsetR3;
}
double prigo::getOffsetR4()
{
	return offsetR4;
}
double prigo::getOffsetS1()
{
	return offsetS1;
}
double prigo::getOffsetS2()
{
	return offsetS2;
}
double prigo::getOffsetS3()
{
	return offsetS3;
}
double prigo::getOffsetS4()
{
	return offsetS4;
}


void prigo::setSHVector(double shx, double shy, double shz)
{
    SHx = shx;
	SHy = shy;
	SHz = shz;
	printf("setSHVector: SH Vector set to [%f, %f, %f]\n", SHx, SHy, SHz);
	return;
}

void prigo::setOVector(double ox, double oy, double oz)
{
    Ox = ox;
	Oy = oy;
	Oz = oz;
	printf("setOVector: O Vector set to [%f, %f, %f]\n", Ox, Oy, Oz);
	return;
}



double prigo::getS1()
{
   return s1;
}

double prigo::getS2()
{
   return s2;
}

double prigo::getS3()
{
   return s3;
}

double prigo::getS4()
{
   return s4;
}

double prigo::getOmegaPos()
{
   return omega_pos;
}

double prigo::getPhiPos()
{
   return phi_pos;
}

void prigo::test()
{
return;
}

int prigo::readParamFile(char* filename)
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
			if      (strcmp(read_str, "SHx")==0){ SHx=read_double;printf("SHx=%lf\n",read_double);}
			else if (strcmp(read_str, "SHy")==0){ SHy=read_double;printf("SHy=%lf\n",read_double);}
			else if (strcmp(read_str, "SHz")==0){ SHz=read_double;printf("SHz=%lf\n",read_double);}
			else if (strcmp(read_str, "Ox")==0){ Ox=read_double;printf("Ox=%lf\n",read_double);}
			else if (strcmp(read_str, "Oy")==0){ Oy=read_double;printf("Oy=%lf\n",read_double);}
			else if (strcmp(read_str, "Oz")==0){ Oz=read_double;printf("Oz=%lf\n",read_double);}
			
			else if (strcmp(read_str, "offsetR1")==0){ offsetR1=read_double;printf("offsetR1=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetR2")==0){ offsetR2=read_double;printf("offsetR2=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetR3")==0){ offsetR3=read_double;printf("offsetR3=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetR4")==0){ offsetR4=read_double;printf("offsetR4=%lf\n",read_double);}
			
			else if (strcmp(read_str, "offsetS1")==0){ offsetS1=read_double;printf("offsetS1=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetS2")==0){ offsetS2=read_double;printf("offsetS2=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetS3")==0){ offsetS3=read_double;printf("offsetS3=%lf\n",read_double);}
			else if (strcmp(read_str, "offsetS4")==0){ offsetS4=read_double;printf("offsetS4=%lf\n",read_double);}

			else if (strcmp(read_str, "l11")==0){ l11=read_double;printf("l11=%lf\n",read_double);}
			else if (strcmp(read_str, "l12")==0){ l12=read_double;printf("l12=%lf\n",read_double);}
			else if (strcmp(read_str, "l13")==0){ l13=read_double;printf("l13=%lf\n",read_double);}
			else if (strcmp(read_str, "l14")==0){ l14=read_double;printf("l14=%lf\n",read_double);}

			else if (strcmp(read_str, "l24")==0){ l24=read_double;printf("l24=%lf\n",read_double);}

			else if (strcmp(read_str, "l31")==0){ l31=read_double;printf("l31=%lf\n",read_double);}
			else if (strcmp(read_str, "l34")==0){ l34=read_double;printf("l34=%lf\n",read_double);}
			else if (strcmp(read_str, "l35")==0){ l35=read_double;printf("l35=%lf\n",read_double);}

			else if (strcmp(read_str, "l41")==0){ l41=read_double;printf("l41=%lf\n",read_double);}
			else if (strcmp(read_str, "l42")==0){ l42=read_double;printf("l42=%lf\n",read_double);}
			else if (strcmp(read_str, "l43")==0){ l43=read_double;printf("l43=%lf\n",read_double);}
			else if (strcmp(read_str, "l44")==0){ l44=read_double;printf("l44=%lf\n",read_double);}
	
			else if (strcmp(read_str, "l51")==0){ l51=read_double;printf("l51=%lf\n",read_double);}
			else if (strcmp(read_str, "l52")==0){ l52=read_double;printf("l52=%lf\n",read_double);}
			else if (strcmp(read_str, "l53")==0){ l53=read_double;printf("l53=%lf\n",read_double);}
			else if (strcmp(read_str, "l54")==0){ l54=read_double;printf("l54=%lf\n",read_double);}
			else if (strcmp(read_str, "l55")==0){ l55=read_double;printf("l55=%lf\n",read_double);}
			else if (strcmp(read_str, "l56")==0){ l56=read_double;printf("l56=%lf\n",read_double);}

			else if (strcmp(read_str, "a5")==0){ a5=read_double;printf("a5=%lf\n",read_double);}
		}
		l21 = sqrt(l35*l35 + l24*l24);
		l22 = sqrt((l31/2)*(l31/2) + l34*l34 + l24*l24);
		l23 = l22;
		l32 = sqrt((l31/2.)*(l31/2.) + (l34+l35)*(l34+l35));
		l33 = l32;

		fclose(pFile);
	}
	return 0;
}

int prigo::writeParamFile(char* filename)
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
		fprintf(pFile, "SHx\t%lf\n",SHx);
		fprintf(pFile, "SHy\t%lf\n",SHy);
		fprintf(pFile, "SHz\t%lf\n",SHz);
		fprintf(pFile, "Ox\t%lf\n",Ox);
		fprintf(pFile, "Oy\t%lf\n",Oy);
		fprintf(pFile, "Oz\t%lf\n",Oz);

		fprintf(pFile, "offsetR1\t%lf\n",offsetR1);


		fprintf(pFile, "offsetR2\t%lf\n",offsetR2);
		fprintf(pFile, "offsetR3\t%lf\n",offsetR3);
		fprintf(pFile, "offsetR4\t%lf\n",offsetR4);

		fprintf(pFile, "offsetS1\t%lf\n",offsetS1);
		fprintf(pFile, "offsetS2\t%lf\n",offsetS2);
		fprintf(pFile, "offsetS3\t%lf\n",offsetS3);
		fprintf(pFile, "offsetS4\t%lf\n",offsetS4);

		fprintf(pFile, "l11\t%lf\n",l11);
		fprintf(pFile, "l12\t%lf\n",l12);
		fprintf(pFile, "l13\t%lf\n",l13);
		fprintf(pFile, "l14\t%lf\n",l14);

		fprintf(pFile, "l24\t%lf\n",l24);

		fprintf(pFile, "l31\t%lf\n",l31);
		fprintf(pFile, "l34\t%lf\n",l34);
		fprintf(pFile, "l35\t%lf\n",l35);

		fprintf(pFile, "l41\t%lf\n",l41);
		fprintf(pFile, "l42\t%lf\n",l42);
		fprintf(pFile, "l43\t%lf\n",l43);
		fprintf(pFile, "l44\t%lf\n",l44);

		fprintf(pFile, "l51\t%lf\n",l51);
		fprintf(pFile, "l52\t%lf\n",l52);
		fprintf(pFile, "l53\t%lf\n",l53);
		fprintf(pFile, "l54\t%lf\n",l54);
		fprintf(pFile, "l55\t%lf\n",l55);
		fprintf(pFile, "l56\t%lf\n",l56);

		fprintf(pFile, "a5\t%lf\n",a5);
		fclose(pFile);
	}
	return 0;
}

int prigo::printParams()
{
		//start writing line by line:
		printf("SHx\t%lf\n",SHx);
		printf("SHy\t%lf\n",SHy);
		printf("SHz\t%lf\n",SHz);
		printf("Ox\t%lf\n",Ox);
		printf("Oy\t%lf\n",Oy);
		printf("Oz\t%lf\n",Oz);

		printf("offsetR1\t%lf\n",offsetR1);
		printf("offsetR2\t%lf\n",offsetR2);
		printf("offsetR3\t%lf\n",offsetR3);
		printf("offsetR4\t%lf\n",offsetR4);

		printf("offsetS1\t%lf\n",offsetS1);
		printf("offsetS2\t%lf\n",offsetS2);
		printf("offsetS3\t%lf\n",offsetS3);
		printf("offsetS4\t%lf\n",offsetS4);

		printf("l11\t%lf\n",l11);
		printf("l12\t%lf\n",l12);
		printf("l13\t%lf\n",l13);
		printf("l14\t%lf\n",l14);

		printf("l24\t%lf\n",l24);

		printf("l31\t%lf\n",l31);
		printf("l34\t%lf\n",l34);
		printf("l35\t%lf\n",l35);

		printf("l41\t%lf\n",l41);
		printf("l42\t%lf\n",l42);
		printf("l43\t%lf\n",l43);
		printf("l44\t%lf\n",l44);

		printf("l51\t%lf\n",l51);
		printf("l52\t%lf\n",l52);
		printf("l53\t%lf\n",l53);
		printf("l54\t%lf\n",l54);
		printf("l55\t%lf\n",l55);
		printf("l56\t%lf\n",l56);

		printf("a5\t%lf\n",a5);
	return 0;
}

int prigo::switchOn_CATIALawFile(char* filename) 
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

int prigo::switchOff_CATIALawFile(void) 
{	
		return fclose(CATIALawFile);
}
int prigo::CATIALawFile_writeLine(double UCS1,double UCS2,double UCS3,double UCS4,double UCS5,double UCS6,double UCS7,double UCS8,double UCS9)
{
	//check if file is open
	if (CATIALawFile != NULL) {
		double UCS[10] = {0.,UCS1,UCS2,UCS3,UCS4,UCS5,UCS6,UCS7,UCS8,UCS9};
		double MCS[10] = {0., 0.,0.,0., 0.,0.,0., 0.,0.,0.};
		int res;
		char errMsg[100], comment[100];
		strcpy (comment,"");
		res = mgi(UCS,MCS,errMsg); //run IK model to get Motor Coords
		if (res != 0){ //if an error was encountered, print behind the line //error:...
			strcpy (comment,"//");
			strcat(comment,errMsg);
		}
        fprintf(CATIALawFile, "%d\t%.4lf\t%.4lf\t%.4lf\t%.4lf %s\n",	CATIALawFile_counter,
																	MCS[1],MCS[2],MCS[3],MCS[4],
																	comment);
		CATIALawFile_counter = CATIALawFile_counter + 10;
		return 0;
	} else {
		return 1;
	}

}

int prigo::logString(char* string)
{
	FILE *filehandle;
	char filename[100] = "PRIGOlog.txt", mode[4]="a";
	filehandle = fopen(filename, mode);
	fputs(string,filehandle);//write to file	
	fclose(filehandle);
	return 0;
}



int prigo::mgi(double *Point_op, double *Point_art, char *msg)
{
	//Assign Operational Coordinates
	double SHx = Point_op[1]; //sample holder vector
	double SHy = Point_op[2];
	double SHz = Point_op[3];
	double CHI = Point_op[4]; //UPPER CASE angles are in DEG (lower case are in rad)
	double PHI = Point_op[5]; 
	double OMEGA = Point_op[6];

	double Ox  = Point_op[7]; //Centre of Interest (described with Frame 0)
	double Oy  = Point_op[8]; 
	double Oz  = Point_op[9];


	//convert to radians (note: angles in lowercase are used later on)
	double chi = CHI/360*2*Pi;
	double omega = OMEGA/360*2*Pi;
	double phi =  PHI/360*2*Pi;
	//set initial value for theta
	double theta = chi;

	//Update calculated lengths.
	l21 = sqrt(l35*l35 + l24*l24);
	l22 = sqrt((l31/2)*(l31/2) + l34*l34 + l24*l24);
	l23 = l22;
	l32 = sqrt((l31/2.)*(l31/2.) + (l34+l35)*(l34+l35));
	l33 = l32;

	//Transfer Matrix 0->1, Rotation by omega, around z-axis
	TypeMatH4 H01; 
	H01[0][0] = cos(omega);		H01[0][1] = sin(omega);		H01[0][2] = 0.;	H01[0][3] = 0.;
	H01[1][0] = -sin(omega);	H01[1][1] = cos(omega);		H01[1][2] = 0.;	H01[1][3] = 0.;
	H01[2][0] = 0.;				H01[2][1] = 0.;				H01[2][2] = 1.;	H01[2][3] = 0.;
	H01[3][0] = 0.;				H01[3][1] = 0.;				H01[3][2] = 0.;	H01[3][3] = 1.;
	//Convert Tip of Tetraeder Oxyz (described above in Frame 0) into Frame 1:
	//Oin1 = H01 * Oin0
	//TypeVectH4 Oin0 = {Ox, Oy, Oz, 1};
	//TypeVectH4 Oin1;
	//MulMatH4VectH4(H01, Oin0, Oin1); //Oin1 = H01 * Oin0;
	TypeVectH4 Oin1 = {Ox, Oy, Oz, 1};

	//X Holds the solution of the 9x9 equation system
	double X[9];

	//declare other variables here, that will be used outside the loop.
	double B1x, B1y, B1z, B2x, B2y, B2z, B3x, B3y, B3z;
	TypeMatH4 H32, H21, H31;

	//Theta Converge Loop
	bool loopcond1 = true;
	int loopcounter1 = 0;	
	while (loopcond1==true) {

		double TipX, TipY, TipZ, d1, d2, d3;
		//calcute tip of pseudo-tetraeder, in function of (theta, phi, SHx, SHy, SHz)
		TipX = SHx*cos(phi) - SHy*sin(phi);
		TipY = SHx*cos(theta)*sin(phi) + SHy*cos(theta)*cos(phi) - SHz*sin(theta) - l11*sin(theta)+l12*cos(theta)+l34;
		TipZ = SHx*sin(theta)*sin(phi) + SHy*sin(theta)*cos(phi) + SHz*cos(theta) + l11*cos(theta)+l12*sin(theta)+l24;

		//Calculate Tetraeder Form (d1, d2, d3 = f(theta)):
		d1 = sqrt(TipX*TipX + (l34+l35-TipY)*(l34+l35-TipY) + TipZ*TipZ);
		d2 = sqrt((l31/2-TipX)*(l31/2-TipX) + TipY*TipY + TipZ*TipZ);
		d3 = sqrt((-l31/2-TipX)*(-l31/2-TipX) + TipY*TipY + TipZ*TipZ);

		//Starting Condition for numeric calculation:
		X[0]=0; X[1]=l35; X[2]=l41+offsetS1+offsetR1; X[3]=l31/2; X[4]=-l34; X[5]=l42+offsetS2+offsetR2; X[6]=-l31/2; X[7]=-l34; X[8]=l42+offsetS3+offsetR3;

		//9x9 Equation Solving Loop:
		bool loopcond2 = true;
		int loopcounter2 = 0;	
		while (loopcond2==true){
			double A[9][9];
			double b[9];
			double y[9];
			int pivot[9];
			//b = f(x)
			b[0] = X[0];
			b[1] = X[3] - l56/2 - sqrt(3.)*(-l54) + sqrt(3.)*X[4];
			b[2] = X[6] + l56/2 + sqrt(3.)*(-l54) - sqrt(3.)*X[7];
			b[3] = (X[0]-Oin1[0])*(X[0]-Oin1[0]) + (X[1]-Oin1[1])*(X[1]-Oin1[1]) + (X[2]-Oin1[2])*(X[2]-Oin1[2]) - d1*d1;
			b[4] = (X[3]-Oin1[0])*(X[3]-Oin1[0]) + (X[4]-Oin1[1])*(X[4]-Oin1[1]) + (X[5]-Oin1[2])*(X[5]-Oin1[2]) - d2*d2;
			b[5] = (X[6]-Oin1[0])*(X[6]-Oin1[0]) + (X[7]-Oin1[1])*(X[7]-Oin1[1]) + (X[8]-Oin1[2])*(X[8]-Oin1[2]) - d3*d3;
			b[6] = (X[6]-X[3])*(X[6]-X[3]) + (X[7]-X[4])*(X[7]-X[4]) + (X[8]-X[5])*(X[8]-X[5]) - l31*l31;
			b[7] = (X[3]-X[0])*(X[3]-X[0]) + (X[4]-X[1])*(X[4]-X[1]) + (X[5]-X[2])*(X[5]-X[2]) - l32*l32;
			b[8] = (X[6]-X[0])*(X[6]-X[0]) + (X[7]-X[1])*(X[7]-X[1]) + (X[8]-X[2])*(X[8]-X[2]) - l33*l33;
			
			//A = Jacobi Matrix of f(x): (Df(x))
			A[0][0] = 1; 
			A[0][1] = 0; 
			A[0][2] = 0;	
			A[0][3] = 0;		
			A[0][4] = 0;		
			A[0][5] = 0;		
			A[0][6] = 0;		
			A[0][7] = 0;		
			A[0][8] = 0;		

			A[1][0] = 0;
			A[1][1] = 0;
			A[1][2] = 0;
			A[1][3] = 1;
			A[1][4] = sqrt(3.);
			A[1][5] = 0;
			A[1][6] = 0;
			A[1][7] = 0;
			A[1][8] = 0;

			A[2][0] = 0;
			A[2][1] = 0;
			A[2][2] = 0;
			A[2][3] = 0;
			A[2][4] = 0;
			A[2][5] = 0;
			A[2][6] = 1;
			A[2][7] = -sqrt(3.);
			A[2][8] = 0;
				
			A[3][0] = 2*(X[0]-Oin1[0]);
			A[3][1] = 2*(X[1]-Oin1[1]);
			A[3][2] = 2*(X[2]-Oin1[2]);
			A[3][3] = 0;
			A[3][4] = 0;
			A[3][5] = 0;
			A[3][6] = 0;
			A[3][7] = 0;
			A[3][8] = 0;

			A[4][0] = 0;
			A[4][1] = 0;
			A[4][2] = 0;
			A[4][3] = 2*(X[3]-Oin1[0]);
			A[4][4] = 2*(X[4]-Oin1[1]);
			A[4][5] = 2*(X[5]-Oin1[2]);
			A[4][6] = 0;
			A[4][7] = 0;
			A[4][8] = 0;

			A[5][0] = 0;		
			A[5][1] = 0;		
			A[5][2] = 0;		
			A[5][3] = 0;		
			A[5][4] = 0;		
			A[5][5] = 0;		
			A[5][6] = 2*(X[6]-Oin1[0]);		
			A[5][7] = 2*(X[7]-Oin1[1]);		
			A[5][8] = 2*(X[8]-Oin1[2]);	

			A[6][0] = 0;		
			A[6][1] = 0;		
			A[6][2] = 0;		
			A[6][3] = 2*(X[3]-X[6]);		
			A[6][4] = 2*(X[4]-X[7]);		
			A[6][5] = 2*(X[5]-X[8]);		
			A[6][6] = 2*(-X[3]+X[6]);		
			A[6][7] = 2*(-X[4]+X[7]);		
			A[6][8] = 2*(-X[5]+X[8]);		

			A[7][0] = 2*(X[0]-X[3]);		
			A[7][1] = 2*(X[1]-X[4]);		
			A[7][2] = 2*(X[2]-X[5]);		
			A[7][3] = 2*(-X[0]+X[3]);		
			A[7][4] = 2*(-X[1]+X[4]);		
			A[7][5] = 2*(-X[2]+X[5]);		
			A[7][6] = 0;		
			A[7][7] = 0;		
			A[7][8] = 0;		

			A[8][0] = 2*(X[0]-X[6]);		
			A[8][1] = 2*(X[1]-X[7]);		
			A[8][2] = 2*(X[2]-X[8]);		
			A[8][3] = 0;		
			A[8][4] = 0;		
			A[8][5] = 0;		
			A[8][6] = 2*(-X[0]+X[6]);		
			A[8][7] = 2*(-X[1]+X[7]);		
			A[8][8] = 2*(-X[2]+X[8]);		

			//Solve equation system (LU Decomposition, LU Solve)
			int decompOK = 0, solveOK = 0;
			decompOK= Doolittle_LU_Decomposition_with_Pivoting(&A[0][0], pivot, 9);
			solveOK = Doolittle_LU_with_Pivoting_Solve(&A[0][0], b, pivot, y, 9);
		
			//Newton Corde (Xn+1 = Xn - yn)
			{
			X[0] -= y[0];
			X[1] -= y[1];
			X[2] -= y[2];
			X[3] -= y[3];
			X[4] -= y[4];
			X[5] -= y[5];
			X[6] -= y[6];
			X[7] -= y[7];
			X[8] -= y[8];
			}
			//Loop condition: If precision better than a nanometer ((10^-6)^2 = 10^-12 [mm]) exit loop
			if (NormeCarreVect(y,9) < 1e-6)
				loopcond2 = false;

			//Stop looping if more than 30 iterations were done, sound alarm.
			loopcounter2++; //Increment Loop Counter
			if (loopcounter2 > 30) {
				loopcond2 = false;
				//printf("Loop 2 (tetraeder) did not converge after 30 iterations\n");
				//printf("norm of y: %f\n", y.Norm1());
				//logString("MGD: Loop 2 (tetraeder) did not converge after 30 iterations\n");
				sprintf(msg, "Loop 2 (9x9 matrix solve) did not converge after 30 iterations\n");
				return 2;
			}

		}
		B1x = X[0];
		B1y = X[1];
		B1z = X[2];
		B2x = X[3];
		B2y = X[4];
		B2z = X[5];
		B3x = X[6];
		B3y = X[7];
		B3z = X[8];

		//Next, the CHI angle is measured in the current posture
		//This cam be calculated as CHI=arcsin((Pr1in1y-Hin1y)/l11), see paper notes for details.

		//For that, Points Pr1in1 and Hin1 must be found,
		//Pr1 and H are easy to determine in Frame 3, so let's find
		//a frame tranformation from 3->1:  H31

		//H31 can be calculated as H31= H21 * H32

		//let's calculate H21:
		H21[0][3] = X[6] + (X[3]-X[6])/2;
		H21[1][3] = X[7] + (X[4]-X[7])/2;
		H21[2][3] = X[8] + (X[5]-X[8])/2;

		H21[0][0] = 2*(X[3]-H21[0][3])/l31;
		H21[1][0] = 2*(X[4]-H21[1][3])/l31;
		H21[2][0] = 2*(X[5]-H21[2][3])/l31;

		H21[0][1] = (X[0]-H21[0][3])/(l34+l35);
		H21[1][1] = (X[1]-H21[1][3])/(l34+l35);
		H21[2][1] = (X[2]-H21[2][3])/(l34+l35);

		H21[0][2] = H21[1][0]*H21[2][1] - H21[2][0]*H21[1][1];
		H21[1][2] = H21[2][0]*H21[0][1] - H21[0][0]*H21[2][1];
		H21[2][2] = H21[0][0]*H21[1][1] - H21[1][0]*H21[0][1];

		H21[3][0] = 0.;
		H21[3][1] = 0.;
		H21[3][2] = 0.;
		H21[3][3] = 1.;
        
		//Now let's go for H32
		H32[0][0] = 1;	H32[0][1]= 0;			H32[0][2]= 0;		H32[0][3]= 0;
		H32[1][0] = 0;	H32[1][1]= sin(theta);		H32[1][2]= cos(theta);	H32[1][3]= l34;
		H32[2][0] = 0;	H32[2][1]= -cos(theta);		H32[2][2]= sin(theta);	H32[2][3]= l24;
		H32[3][0] = 0;	H32[3][1]= 0;			H32[3][2]= 0;		H32[3][3]= 1;

		//H31 = H21 * H32
		MulMatMatH4(H21,H32, H31); //H31 = H21 * H32;

		TypeVectH4 Pr1in3 = {0,0,l12,1};
		TypeVectH4 Pr1in1;
		TypeVectH4 Hin3 = {0,-l11,l12,1};		
		TypeVectH4 Hin1;		
		MulMatH4VectH4(H31,Pr1in3, Pr1in1); //Pr1in1 = H31 * [0;0;l12;1];
		MulMatH4VectH4(H31,Hin3, Hin1);     //Hin1   = H31 * [0;-l11;l12;1];

		//so here we have chi as it is in this posture.
		double chi_measured = atan2(Pr1in1[1]-Hin1[1],-(Pr1in1[2]-Hin1[2]))-atan2(0,1); 

		//find out the error from the target chi
 		double chierror = chi - chi_measured;

		//if the error is below 10-6 rad we're happy, and we'll stop looping
		if (fabs(chierror) < 10e-6)
			loopcond1 = false;
		//Stop looping if more than 50 iterations were done
		else {
			if (loopcounter1 > 50){
			loopcond1 = false;
				sprintf(msg, "Loop 1 (chi find) did not converge after 50 iterations\n");
				return 1;
			}
			else {
				loopcounter1++; //Increment Loop Counter
			
				//Calculate Theta for next run
				theta = theta + 0.9*chierror;
			}
		}
	}
	//---------------------------------------------------------------------------------------
	//Slider Location Calculations (Location of P1, P2, P3)
	//Construct P1, P2, P3 positions in Frame 1
	double P1x = 0;		double P2x = l56/2;	double P3x = -l56/2;
	double P1y = l55;	double P2y = -l54;	double P3y = -l54;
	double P1z;			double P2z;			double P3z;
	{
		double a,b,c,d;
		a = 1;
		b = -2*B1z;
		c = B1z*B1z + (B1x-P1x)*(B1x-P1x) + (B1y-P1y)*(B1y-P1y) - l41*l41;
		d = sqrt(b*b - 4*a*c);
		P1z = (-b -d)/(2*a);
		
		a = 1;
		b = -2*B2z;
		c = B2z*B2z + (B2x-P2x)*(B2x-P2x) + (B2y-P2y)*(B2y-P2y) - l42*l42;
		d = sqrt(b*b - 4*a*c);
		P2z = (-b -d)/(2*a);
		
		a = 1;
		b = -2*B3z;
		c = B3z*B3z + (B3x-P3x)*(B3x-P3x) + (B3y-P3y)*(B3y-P3y) - l43*l43;
		d = sqrt(b*b - 4*a*c);
		P3z = (-b -d)/(2*a);
	}
	//TypeVectH4 P1in1 = {P1x,P1y,P1z,1};
	//TypeVectH4 P2in1 = {P2x,P2y,P2z,1};
	//TypeVectH4 P3in1 = {P3x,P3y,P3z,1};

	//---------------------------------------------------------------------------------------
	//Slider Location Calculations (for Slider 4)
	//TypeVectH4 Pr2in3 = {0, l13, l12, 1};
	
	//Calculte B4in5, starting from B4in3
	//B4in5 = H15*H21*H32*B4in3;
	TypeVectH4 B4in3 = {0, l13, l12-l14, 1};
	TypeVectH4 B4in2;
	TypeVectH4 B4in1;
	MulMatH4VectH4(H32,B4in3, B4in2); //B4in2 = H32 * B4in3;
	MulMatH4VectH4(H21,B4in2, B4in1); //B4in1 = H21 * B4in2;

	//Construct Transision Matrix 1->5 (H15)
	//Rotation Matrix 1->5
	TypeMatH4 H15;
	H15[0][0]= 1;	H15[0][1]= 0;		H15[0][2]= 0;		H15[0][3]= 0;
	H15[1][0]= 0;	H15[1][1]= sin(a5);	H15[1][2]= cos(a5);	H15[1][3]= -l51*cos(a5)+l52*sin(a5);
	H15[2][0]= 0;	H15[2][1]=-cos(a5);	H15[2][2]= sin(a5);	H15[2][3]= -l53-l51*sin(a5)-l52*cos(a5);
	H15[3][0]= 0;	H15[3][1]= 0;		H15[3][2]= 0;		H15[3][3]= 1;

	//invert H15 -> H51
	TypeMatH4 H51;
	H51[0][0]= 1;	H51[0][1]= 0;		H51[0][2]= 0;		H51[0][3]= 0;
	H51[1][0]= 0;	H51[1][1]= sin(a5);	H51[1][2]=-cos(a5);	H51[1][3]= -l52 - l53*cos(a5);
	H51[2][0]= 0;	H51[2][1]= cos(a5);	H51[2][2]= sin(a5);	H51[2][3]= +l51 + l53*sin(a5);
	H51[3][0]= 0;	H51[3][1]= 0;		H51[3][2]= 0;		H51[3][3]= 1;
	
	//use H15 to get to B4in5
	TypeVectH4 B4in5;
	MulMatH4VectH4(H15,B4in1, B4in5); //B4in5 = H15 * B4in1;

	//Intersect Slider with Bielle: (B5y in frame 5)
	double B5y= 0.5*(2*B4in5[1] - sqrt(4*B4in5[1]*B4in5[1]-4*(B4in5[0]*B4in5[0]+B4in5[1]*B4in5[1]+B4in5[2]*B4in5[2]-l44*l44)));
	//TypeVectH4 B5in5 = {0, B5y, 0, 1};
	//TypeVectH4 B5in1;
	//MulMatH4VectH4(H51, B5in5, B5in1); //B5in1 = H51 * B5in5;

	//---------------------------------------------------------------------------------------

	//export parameters	
	Point_art[1] = -(P1z-offsetS1-offsetR1);
	Point_art[2] = -(P2z-offsetS2-offsetR2);
	Point_art[3] = -(P3z-offsetS3-offsetR3);
	Point_art[4] = -(B5y-offsetS4-offsetR4);
	Point_art[5] = PHI;
	Point_art[6] = OMEGA;

	Point_art[7] = Ox;
	Point_art[8] = Oy;
	Point_art[9] = Oz;


	sprintf(msg, "ok");
	return 0;
}

//overloaded mgi function without msg feedback
int prigo::mgi(double *Point_op, double *Point_art)
{
	char msg[1000];
	return mgi(Point_op, Point_art, msg);
}




int prigo::mgd(double *Point_art, double *Point_op, char *msg)
{
	//Assign Articular Coordinates
	double s1  = Point_art[1]; //Slider positions
	double s2  = Point_art[2];
	double s3  = Point_art[3];
	double s4  = Point_art[4]; 
	double PHIMOTOR = Point_art[5]; //UPPER CASE angles are in DEG (lower case are in rad)
	double OMEGAMOTOR = Point_art[6];

	double Ox  = Point_art[7]; //Centre of Interest (described with Frame 0)
	double Oy  = Point_art[8]; 
	double Oz  = Point_art[9];

	//convert to radians (note: angles in lowercase are used later on)
	double omegamotor = OMEGAMOTOR/360*2*Pi;
	double phimotor =  PHIMOTOR/360*2*Pi;

	//Update calculated lengths.
	l21 = sqrt(l35*l35 + l24*l24);
	l22 = sqrt((l31/2)*(l31/2) + l34*l34 + l24*l24);
	l23 = l22;
	l32 = sqrt((l31/2.)*(l31/2.) + (l34+l35)*(l34+l35));
	l33 = l32;

	//X Holds the solution of the 9x9 equation system
	double X[9];

	//declare other variables here, that will be used outside the loop.
	double B1x, B1y, B1z, B2x, B2y, B2z, B3x, B3y, B3z;
	
	//Get P1, P2, P3, B5 Positions:
	TypeVectH4 P1in1 = {0, l55, -s1+offsetR1+offsetS1, 1.}; 
	TypeVectH4 P2in1 = {l56/2, -l54, -s2+offsetR2+offsetS2, 1.}; 
	TypeVectH4 P3in1 = {-l56/2, -l54, -s3+offsetR3+offsetS3, 1.}; 
	TypeVectH4 B5in5 = {0., -s4+offsetR4+offsetS4, 0., 1.};

	//Starting Condition for numeric calculation:
	X[0]=0; X[1]=l35; X[2]=P1in1[2]+l41; X[3]=l31/2; X[4]=-l34; X[5]=P2in1[2]+l42; X[6]=-l31/2; X[7]=-l34; X[8]=P3in1[2]+l43;
//printf("X0 X1 X2 X3: %lf, %lf, %lf, %lf,%lf, %lf, %lf, %lf,%lf, %lf\n", X[0],X[1],X[2],X[3], X[4],X[5],X[6],X[7],X[8],X[9]);

	//9x9 Equation Solving Loop:
	bool loopcond2 = true;
	int loopcounter2 = 0;	
	while (loopcond2==true){
		double A[9][9];
		double b[9];
		double y[9];
		int pivot[9];
		//b = f(x)
		b[0] = X[0];
		b[1] = X[3] - P2in1[0] - sqrt(3.)*P2in1[1] + sqrt(3.)*X[4];
		b[2] = X[6] - P3in1[0] + sqrt(3.)*P3in1[1] - sqrt(3.)*X[7];
		b[3] = (X[0]-P1in1[0])*(X[0]-P1in1[0]) + (X[1]-P1in1[1])*(X[1]-P1in1[1]) + (X[2]-P1in1[2])*(X[2]-P1in1[2]) - l41*l41;
		b[4] = (X[3]-P2in1[0])*(X[3]-P2in1[0]) + (X[4]-P2in1[1])*(X[4]-P2in1[1]) + (X[5]-P2in1[2])*(X[5]-P2in1[2]) - l42*l42;
		b[5] = (X[6]-P3in1[0])*(X[6]-P3in1[0]) + (X[7]-P3in1[1])*(X[7]-P3in1[1]) + (X[8]-P3in1[2])*(X[8]-P3in1[2]) - l43*l43;
		b[6] = (X[6]-X[3])*(X[6]-X[3]) + (X[7]-X[4])*(X[7]-X[4]) + (X[8]-X[5])*(X[8]-X[5]) - l31*l31;
		b[7] = (X[3]-X[0])*(X[3]-X[0]) + (X[4]-X[1])*(X[4]-X[1]) + (X[5]-X[2])*(X[5]-X[2]) - l32*l32;
		b[8] = (X[6]-X[0])*(X[6]-X[0]) + (X[7]-X[1])*(X[7]-X[1]) + (X[8]-X[2])*(X[8]-X[2]) - l33*l33;
		
		//A = Jacobi Matrix of f(x): (Df(x))
		A[0][0] = 1; 
		A[0][1] = 0; 
		A[0][2] = 0;	
		A[0][3] = 0;		
		A[0][4] = 0;		
		A[0][5] = 0;		
		A[0][6] = 0;		
		A[0][7] = 0;		
		A[0][8] = 0;		

		A[1][0] = 0;
		A[1][1] = 0;
		A[1][2] = 0;
		A[1][3] = 1;
		A[1][4] = sqrt(3.);
		A[1][5] = 0;
		A[1][6] = 0;
		A[1][7] = 0;
		A[1][8] = 0;

		A[2][0] = 0;
		A[2][1] = 0;
		A[2][2] = 0;
		A[2][3] = 0;
		A[2][4] = 0;
		A[2][5] = 0;
		A[2][6] = 1;
		A[2][7] = -sqrt(3.);
		A[2][8] = 0;
			
		A[3][0] = 2*(X[0]-P1in1[0]);
		A[3][1] = 2*(X[1]-P1in1[1]);
		A[3][2] = 2*(X[2]-P1in1[2]);
		A[3][3] = 0;
		A[3][4] = 0;
		A[3][5] = 0;
		A[3][6] = 0;
		A[3][7] = 0;
		A[3][8] = 0;

		A[4][0] = 0;
		A[4][1] = 0;
		A[4][2] = 0;
		A[4][3] = 2*(X[3]-P2in1[0]);
		A[4][4] = 2*(X[4]-P2in1[1]);
		A[4][5] = 2*(X[5]-P2in1[2]);
		A[4][6] = 0;
		A[4][7] = 0;
		A[4][8] = 0;

		A[5][0] = 0;		
		A[5][1] = 0;		
		A[5][2] = 0;		
		A[5][3] = 0;		
		A[5][4] = 0;		
		A[5][5] = 0;		
		A[5][6] = 2*(X[6]-P3in1[0]);		
		A[5][7] = 2*(X[7]-P3in1[1]);		
		A[5][8] = 2*(X[8]-P3in1[2]);	

		A[6][0] = 0;		
		A[6][1] = 0;		
		A[6][2] = 0;		
		A[6][3] = 2*(X[3]-X[6]);		
		A[6][4] = 2*(X[4]-X[7]);		
		A[6][5] = 2*(X[5]-X[8]);		
		A[6][6] = 2*(-X[3]+X[6]);		
		A[6][7] = 2*(-X[4]+X[7]);		
		A[6][8] = 2*(-X[5]+X[8]);		

		A[7][0] = 2*(X[0]-X[3]);		
		A[7][1] = 2*(X[1]-X[4]);		
		A[7][2] = 2*(X[2]-X[5]);		
		A[7][3] = 2*(-X[0]+X[3]);		
		A[7][4] = 2*(-X[1]+X[4]);		
		A[7][5] = 2*(-X[2]+X[5]);		
		A[7][6] = 0;		
		A[7][7] = 0;		
		A[7][8] = 0;		

		A[8][0] = 2*(X[0]-X[6]);		
		A[8][1] = 2*(X[1]-X[7]);		
		A[8][2] = 2*(X[2]-X[8]);		
		A[8][3] = 0;		
		A[8][4] = 0;		
		A[8][5] = 0;		
		A[8][6] = 2*(-X[0]+X[6]);		
		A[8][7] = 2*(-X[1]+X[7]);		
		A[8][8] = 2*(-X[2]+X[8]);		

		//Solve equation system (LU Decomposition, LU Solve)
		int decompOK = 0, solveOK = 0;
		decompOK= Doolittle_LU_Decomposition_with_Pivoting(&A[0][0], pivot, 9);
		solveOK = Doolittle_LU_with_Pivoting_Solve(&A[0][0], b, pivot, y, 9);
	
		//Newton Corde (Xn+1 = Xn - yn)
		{
		X[0] -= y[0];
		X[1] -= y[1];
		X[2] -= y[2];
		X[3] -= y[3];
		X[4] -= y[4];
		X[5] -= y[5];
		X[6] -= y[6];
		X[7] -= y[7];
		X[8] -= y[8];
		}
		//Loop condition: If precision better than a nanometer ((10^-6)^2 = 10^-12 [mm]) exit loop
		if (NormeCarreVect(y,9) < 1e-12)
			loopcond2 = false;

		//Stop looping if more than 30 iterations were done, sound alarm.
		loopcounter2++; //Increment Loop Counter
		if (loopcounter2 > 30) {
			loopcond2 = false;
			//printf("Loop 2 (tetraeder) did not converge after 30 iterations\n");
			//printf("norm of y: %f\n", y.Norm1());
			//logString("MGD: Loop 2 (tetraeder) did not converge after 30 iterations\n");
			sprintf(msg, "Loop 2 (9x9 matrix solve) did not converge after 30 iterations\n");
			return 2;
		}

	}
	B1x = X[0];
	B1y = X[1];
	B1z = X[2];
	B2x = X[3];
	B2y = X[4];
	B2z = X[5];
	B3x = X[6];
	B3y = X[7];
	B3z = X[8];

//printf("B1x B1y B1z: %lf, %lf, %lf\n", B1x, B1y, B1z);
//printf("B2x B2y B2z: %lf, %lf, %lf\n", B2x, B2y, B2z);
//printf("B3x B3y B3z: %lf, %lf, %lf\n", B3x, B3y, B3z);

	//----------------------------------------------------------------------
	//Calculate Transition Matrix of Orion Table (Frames 1<->2)
	//B3B2 = [B2x;B2y;B2z]-[B3x;B3y;B3z];
	//B23 = [B3x;B3y;B3z] + 0.5* B3B2; %middle between points B2, B3

	//Next, we'd like to measure the CHI angle in the current posture.
	//this can be calculated as CHI = arcsin ((Pr1yin1-Hyin1)/l11)

	//but first Points Pr1in1 and Hin1 must be found.
	//Pr1 an H are easy to determine in Frame 3
	//so let's then use a frame tranformation from 3->1: H31

	//H31 can be decomposed into H31= H21 * H32

	// so let's calculate H21
	//Calculate Transition Matrix of Orion Table (Frame 2->1: H21)
	TypeMatH4 H21;
	H21[0][3] = X[6] + (X[3]-X[6])/2;
	H21[1][3] = X[7] + (X[4]-X[7])/2;
	H21[2][3] = X[8] + (X[5]-X[8])/2;

	H21[0][0] = 2*(X[3]-H21[0][3])/l31;
	H21[1][0] = 2*(X[4]-H21[1][3])/l31;
	H21[2][0] = 2*(X[5]-H21[2][3])/l31;

	H21[0][1] = (X[0]-H21[0][3])/(l34+l35);
	H21[1][1] = (X[1]-H21[1][3])/(l34+l35);
	H21[2][1] = (X[2]-H21[2][3])/(l34+l35);

	H21[0][2] = H21[1][0]*H21[2][1] - H21[2][0]*H21[1][1];
	H21[1][2] = H21[2][0]*H21[0][1] - H21[0][0]*H21[2][1];
	H21[2][2] = H21[0][0]*H21[1][1] - H21[1][0]*H21[0][1];

	H21[3][0] = 0.;
	H21[3][1] = 0.;
	H21[3][2] = 0.;
	H21[3][3] = 1.;

	//Now we'll try to find the posture of the Swing. The Swing position depends on the positon of slider4.
	//We must follow the kinematic chain from slider4.

	//Point B4 can be calculated in Frame 2:
	//B4 is the intersection of:
	// - a sphere around B5 with radius l44
	// - the plane x=0 in frame 2
	// - a sphere around P4 with radius length(vector(P4,B4))
	//There are two solutions, use the one with the larger y value. 

	//Calculate B5in2: B5in2=H12*H51*B5in5 (We know B5in5 from far above)
	TypeMatH4 H51;
	H51[0][0]= 1;	H51[0][1]= 0;		H51[0][2]= 0;		H51[0][3]= 0;
	H51[1][0]= 0;	H51[1][1]= sin(a5);	H51[1][2]=-cos(a5);	H51[1][3]= - l52 - l53*cos(a5);
	H51[2][0]= 0;	H51[2][1]= cos(a5);	H51[2][2]= sin(a5);	H51[2][3]= l51+  l53*sin(a5);
	H51[3][0]= 0;	H51[3][1]= 0;		H51[3][2]= 0;		H51[3][3]= 1;

//printf("a5: %lf\n", a5);
	
	TypeMatH4 H12;
	InvMatH4(H21, H12);
//printf("H12: %lf %lf %lf %lf\n", H12[0][0],H12[0][1],H12[0][2],H12[0][3]);
//printf("H12: %lf %lf %lf %lf\n", H12[1][0],H12[1][1],H12[1][2],H12[1][3]);
//printf("H12: %lf %lf %lf %lf\n", H12[2][0],H12[2][1],H12[2][2],H12[2][3]);
//printf("H12: %lf %lf %lf %lf\n", H12[3][0],H12[3][1],H12[3][2],H12[3][3]);

//printf("H21: %lf %lf %lf %lf\n", H21[0][0],H21[0][1],H21[0][2],H21[0][3]);
//printf("H21: %lf %lf %lf %lf\n", H21[1][0],H21[1][1],H21[1][2],H21[1][3]);
//printf("H21: %lf %lf %lf %lf\n", H21[2][0],H21[2][1],H21[2][2],H21[2][3]);
//printf("H21: %lf %lf %lf %lf\n", H21[3][0],H21[3][1],H21[3][2],H21[3][3]);
	
	TypeMatH4 H52;
	MulMatMatH4(H12, H51, H52); //H52 = H12 * H51;

	TypeVectH4 B5in2;
	MulMatH4VectH4(H52,B5in5, B5in2); //B5in2 = H52 * B5in5;

	//Determine P4in2:
	TypeVectH4 P4in2 = {0, l34, l24,1};

	//Get length lP4B4:
	double lP4B4 = sqrt(l13*l13+(l12-l14)*(l12-l14));
	
	//Calculate B4in2:   f(lP4B4, B5in2, P4in2) - Bielle 
	TypeVectH4 B4in2;
	//because we have a few local Hilfsvariabeln (R1, R2, d, d1, a, eB5P4in2, eB5P490in2)
	//keep these in a closed namespace. B4in2 stays outside, because we need it further on.
	{
		double R1;
		R1 = sqrt(l44*l44 - (B5in2[0]-P4in2[0])*(B5in2[0]-P4in2[0]));
		double R2;
		R2 = lP4B4;
		double d;
		d = sqrt((B5in2[1]-P4in2[1])*(B5in2[1]-P4in2[1]) + (B5in2[2]-P4in2[2])*(B5in2[2]-P4in2[2]));
		//CHECK VALIDITY:
		if (d*d >= (R1+R2)*(R1+R2)){
			//-> Circles to far apart
			sprintf(msg, "Slider 4 too retracted in. Distance d:%lf\n", (d-R1-R2));
			return 3;
		}
		if (d*d <= (R1-R2)*(R1-R2)){
			//-> Circles to close together
			sprintf(msg, "Slider 4 too far out. Distance d:%lf\n", (sqrt(R1-R2)-d));
			return 3;
		}
		if (d*d == (R1+R2)*(R1+R2)){
			//-> Singularity
			sprintf(msg, "Slider 4:Singularity (pleuelstange points through theta axis)\n");
			return 3;
		}
		//if all valid, continue on:
		double d1;
		d1 = (d*d - R2*R2 + R1*R1)/(2.*d);
		double a;
		a = 1./(2.*d)*sqrt(4.*d*d*R1*R1-(d*d-R2*R2+R1*R1)*(d*d-R2*R2+R1*R1));
		double eB5P4in2[3];
		eB5P4in2[0] = (P4in2[0]-B5in2[0])/d;
		eB5P4in2[1] = (P4in2[1]-B5in2[1])/d;
		eB5P4in2[2] = (P4in2[2]-B5in2[2])/d;
		double eB5P490in2[3];
		eB5P490in2[0] = eB5P4in2[0];
		eB5P490in2[1] = eB5P4in2[2];
		eB5P490in2[2] = - eB5P4in2[1];
		//VORZEICHEN: which solution to take (+1 or -1)
		double VORZEICHEN = 1.;
		
		//Einsetzen:
		B4in2[0] = P4in2[0];
		B4in2[1] = B5in2[1] + d1* eB5P4in2[1] + VORZEICHEN*a*eB5P490in2[1];
		B4in2[2] = B5in2[2] + d1* eB5P4in2[2] + VORZEICHEN*a*eB5P490in2[2];
		/*B4in2[0] = P4in2[0];

		B4in2[1] = (B5in2[1]*B5in2[1]*B5in2[1] - B5in2[1]*B5in2[1]*P4in2[1] + 
		          B5in2[1]*(B5in2[0]*B5in2[0] - l44*l44 + lP4B4*lP4B4 - 
		          P4in2[0]*P4in2[0] - P4in2[1]* P4in2[1] + (B5in2[2] - P4in2[2])*(B5in2[2] - P4in2[2])) + 
		          P4in2[1]*(-B5in2[0]*B5in2[0] + l44*l44 - 
		          lP4B4*lP4B4 + P4in2[0]*P4in2[0] + P4in2[1]*P4in2[1] + 
		          (B5in2[2] - P4in2[2])*(B5in2[2] - P4in2[2])) + 
		          sqrt(-((B5in2[2] - P4in2[2])*(B5in2[2] - P4in2[2])*(B5in2[0]*B5in2[0]*B5in2[0]*B5in2[0] + 
		          B5in2[1]*B5in2[1]*B5in2[1]*B5in2[1] + B5in2[2]*B5in2[2]*B5in2[2]*B5in2[2] - 2*B5in2[2]*B5in2[2]*l44*l44 + 
		          l44*l44*l44*l44 - 2*B5in2[2]*B5in2[2]*lP4B4*lP4B4 - 
		          2*l44*l44*lP4B4*lP4B4 + lP4B4*lP4B4*lP4B4*lP4B4 + 
		          2*B5in2[2]*B5in2[2]*P4in2[0]*P4in2[0] + 2*l44*l44*P4in2[0]*P4in2[0] - 
		          2*lP4B4*lP4B4*P4in2[0]*P4in2[0] + P4in2[0]*P4in2[0]*P4in2[0]*P4in2[0] - 
		          4*B5in2[1]*B5in2[1]*B5in2[1]*P4in2[1] + 2*B5in2[2]*B5in2[2]*
		          P4in2[1]*P4in2[1] - 2*l44*l44*P4in2[1]*P4in2[1] - 
		          2*lP4B4*lP4B4*P4in2[1]*P4in2[1] + 2*P4in2[0]*P4in2[0]*
		          P4in2[1]*P4in2[1] + P4in2[1]*P4in2[1]*P4in2[1]*P4in2[1] + 2*B5in2[0]*B5in2[0]*
		          (-l44*l44 + lP4B4*lP4B4 - P4in2[0]*P4in2[0] + 
		          (B5in2[1] - P4in2[1])*(B5in2[1] - P4in2[1]) + 
		          (B5in2[2] - P4in2[2])*(B5in2[2] - P4in2[2])) - 4*B5in2[1]*P4in2[1]*
		          (-l44*l44 - lP4B4*lP4B4 + P4in2[0]*P4in2[0] + 
		          P4in2[1]*P4in2[1] + (B5in2[2] - P4in2[2])*(B5in2[2] - P4in2[2])) + 
		          2*B5in2[1]*B5in2[1]*(-l44*l44 - lP4B4*lP4B4 + 
		          P4in2[0]*P4in2[0] + 3*P4in2[1]*P4in2[1] + 
		          (B5in2[2] - P4in2[2])*(B5in2[2] - P4in2[2])) - 4*B5in2[2]*
		          (B5in2[2]*B5in2[2] - l44*l44 - lP4B4*lP4B4 + 
		          P4in2[0]*P4in2[0] + P4in2[1]*P4in2[1])*P4in2[2] + 
		          2*(3*B5in2[2]*B5in2[2] - l44*l44 - lP4B4*lP4B4 + 
		          P4in2[0]*P4in2[0] + P4in2[1]*P4in2[1])*P4in2[2]*P4in2[2] - 
		          4*B5in2[2]*P4in2[2]*P4in2[2]*P4in2[2] + P4in2[2]*P4in2[2]*P4in2[2]*P4in2[2]))))/
		          (2*((B5in2[1] - P4in2[1])*(B5in2[1] - P4in2[1]) + (B5in2[2] - P4in2[2])*(B5in2[2] - P4in2[2])));

		B4in2[2] = B5in2[2] + sqrt(-B4in2[1]*B4in2[1] - B5in2[0]*B5in2[0] + 
		         2*B4in2[1]*B5in2[1] - B5in2[1]*B5in2[1] + l44*l44);

		B4in2[3] = 1.;*/
	}//Namespace finished.

	//--------------
	//Transfer Matrix 1->0, Rotation by omega, around z-axis
	TypeMatH4 H10; 
	H10[0][0] = cos(omegamotor);		H10[0][1] = -sin(omegamotor);	H10[0][2] = 0.;	H10[0][3] = 0.;
	H10[1][0] = sin(omegamotor);		H10[1][1] = cos(omegamotor);	H10[1][2] = 0.;	H10[1][3] = 0.;
	H10[2][0] = 0.;				H10[2][1] = 0.;			H10[2][2] = 1.;	H10[2][3] = 0.;
	H10[3][0] = 0.;				H10[3][1] = 0.;			H10[3][2] = 0.;	H10[3][3] = 1.;

	//Transfer Matrix 2->1, Transition Matrix of Orion Table 
	//was calculated above

	//Transfer Matrix 3->2, Orion to Swing
	TypeMatH4 H32;
	double H3222 = (B4in2[2]*l12 + B4in2[1]*l13 - B4in2[2]*l14 - l12*l24 + l14*l24 - l13*l34)/(l12*l12 + l13*l13 - 2*l12*l14 + l14*l14);
	double H3223 = (B4in2[1]*l12 - B4in2[2]*l13 - B4in2[1]*l14 + l13*l24 - l12*l34 + l14*l34)/(l12*l12 + l13*l13 - 2*l12*l14 + l14*l14);
	H32[0][0] = 1;        H32[0][1] =      0;       H32[0][2] =     0;       H32[0][3] = P4in2[0];
	H32[1][0] = 0;        H32[1][1] =  H3222;       H32[1][2] = H3223;       H32[1][3] = P4in2[1];
	H32[2][0] = 0;        H32[2][1] = -H3223;       H32[2][2] = H3222;       H32[2][3] = P4in2[2];
	H32[3][0] = 0;        H32[3][1] =      0;       H32[3][2] =     0;       H32[3][3] =        1; 

	//Transfer Matrix 4->3, Phi Rotation
	TypeMatH4 H43;
	H43[0][0] = cos(phimotor);   H43[0][1] = -sin(phimotor);   H43[0][2] = 0;       H43[0][3] = 0;
	H43[1][0] = 0;               H43[1][1] = 0;                H43[1][2] = -1;      H43[1][3] = -l11;
	H43[2][0] = sin(phimotor);   H43[2][1] = cos(phimotor);    H43[2][2] = 0;       H43[2][3] = l12;
	H43[3][0] = 0;               H43[3][1] = 0;                H43[3][2] = 0;       H43[3][3] = 1; 

	//Transfer Matrix 4->0, (H40 = H10*H21*H32*H43)
	TypeMatH4 H31;
	MulMatMatH4(H21, H32, H31); //H31 = H21 * H32; //H31 is used further down!
	TypeMatH4 H30;
	MulMatMatH4(H10, H31, H30); //H30 = H10 * H31;
	TypeMatH4 H40;
	MulMatMatH4(H30, H43, H40); //H40 = H30 * H43;

	//Transfer Matrix 0->4, ( H04 = inv(H40) )
	TypeMatH4 H04;
	InvMatH4(H40, H04);

	//Transfer Matrix 4->1, (H41 = H21*H32*H43 = H31*H43)
	TypeMatH4 H41;
	MulMatMatH4(H31, H43, H41); //H41 = H31 * H43;

	//Transfer Matrix 1->4, ( H14 = inv(H41) )
	TypeMatH4 H14;
	InvMatH4(H41, H14);
//printf("H04: %lf %lf %lf %lf\n", H32[0][0],H32[0][1],H32[0][2],H32[0][3]);
//printf("H04: %lf %lf %lf %lf\n", H32[1][0],H32[1][1],H32[1][2],H32[1][3]);
//printf("H04: %lf %lf %lf %lf\n", H32[2][0],H32[2][1],H32[2][2],H32[2][3]);
//printf("H04: %lf %lf %lf %lf\n", H32[3][0],H32[3][1],H32[3][2],H32[3][3]);
	
	//Calculate Oin4 from Oin0 (Oin4 = H04*Oin0)
	TypeVectH4 Oin1 = {Ox, Oy, Oz, 1};
	TypeVectH4 Oin4;
	MulMatH4VectH4(H14, Oin1, Oin4); //Oin4 = H14 * Oin1;


	//Calculate CHI
	//need Pr1in1 & Hin1:
	TypeVectH4 Pr1in3 = {0, 0, l12, 1};
	TypeVectH4 Hin3   = {0, -l11, l12, 1};

	TypeVectH4 Pr1in1;
	MulMatH4VectH4(H31, Pr1in3, Pr1in1); //Pr1in1 = H31 * Pr1in3;
	TypeVectH4 Hin1;
	MulMatH4VectH4(H31, Hin3, Hin1); //Pr1in1 = H31 * Pr1in3;

	//measure chi in Frame 1
	double chi_measured;
	(Pr1in1[2]-Hin1[2]) < 0 ? 
		chi_measured = asin((Pr1in1[1]-Hin1[1])/l11)
		: chi_measured = ((Pi/2) + atan((Pr1in1[2]-Hin1[2])/l11));
	double CHI = chi_measured*360/2/Pi;

	double PHI = phimotor*360/2/Pi;
	double OMEGA = omegamotor*360/2/Pi;



	//export parameters
	Point_op[1] = Oin4[0];		//SHx
	Point_op[2] = Oin4[1];		//SHy
	Point_op[3] = Oin4[2];		//SHz
	Point_op[4] = CHI;		//CHI
	Point_op[5] = PHI;		//PHI
	Point_op[6] = OMEGA;		//OMEGA
	Point_op[7] = Ox;		//Ox
	Point_op[8] = Oy;		//Oy
	Point_op[9] = Oz; 		//Oz

	sprintf(msg, "ok");
	return 0;
}

//overloaded mgi function without msg feedback
int prigo::mgd(double *Point_art, double *Point_op)
{
	char msg[1000];
	return mgd(Point_art, Point_op, msg);
}
