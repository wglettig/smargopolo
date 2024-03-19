/***********************************************************************
 *
 * File name: prigoIK.cpp
 * Author   : Wayne Glettig, Marco Salathe
 * Version  : Feb. 2012
 *
 * Module that contains the geometric model and does the inverse kinematics
 *
 **********************************************************************/

#include <DLCInterface.hpp>
#include "prigoGM/prigo.h"

INITIALIZE_BEGIN
	ALLOCATE_EXTRA(0, 1, prigo);

	//read parameter file from PARAMS File (prigoIK.xmp)
	//pass PARAMeters to Prigo GM.
	EXTRA(0,0).offsetR1 = PARAM("offsetR1",0);
	EXTRA(0,0).offsetR2 = PARAM("offsetR2",0);
	EXTRA(0,0).offsetR3 = PARAM("offsetR3",0);
	EXTRA(0,0).offsetR4 = PARAM("offsetR4",0);
	EXTRA(0,0).offsetS1 = PARAM("offsetS1",0);
	EXTRA(0,0).offsetS2 = PARAM("offsetS2",0);
	EXTRA(0,0).offsetS3 = PARAM("offsetS3",0);
	EXTRA(0,0).offsetS4 = PARAM("offsetS4",0);
	EXTRA(0,0).l11 = PARAM("l11",0);
	EXTRA(0,0).l12 = PARAM("l12",0);
	EXTRA(0,0).l13 = PARAM("l13",0);
	EXTRA(0,0).l14 = PARAM("l14",0);
	EXTRA(0,0).l24 = PARAM("l24",0);
	EXTRA(0,0).l31 = PARAM("l31",0);
	EXTRA(0,0).l34 = PARAM("l34",0);
	EXTRA(0,0).l35 = PARAM("l35",0);
	EXTRA(0,0).l41 = PARAM("l41",0);
	EXTRA(0,0).l42 = PARAM("l42",0);
	EXTRA(0,0).l43 = PARAM("l43",0);
	EXTRA(0,0).l44 = PARAM("l44",0);
	EXTRA(0,0).l51 = PARAM("l51",0);
	EXTRA(0,0).l52 = PARAM("l52",0);
	EXTRA(0,0).l53 = PARAM("l53",0);
	EXTRA(0,0).l54 = PARAM("l54",0);
	EXTRA(0,0).l55 = PARAM("l55",0);
	EXTRA(0,0).l56 = PARAM("l56",0);
	EXTRA(0,0).a5  = PARAM("a5",0);
	EXTRA(0,0).printParams();

	//run geometrical model to test if it works;
	double UCS[10] = {0., 0.,0.,20., 0.,0.,0., 0.,0.,275.};
	double MCS[10] = {0., 0.,0.,0., 0.,0.,0., 0.,0.,0.};
	char msg[200];
	int errNo;
	printf("IK Model: Sending:\n");
	printf("UCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", UCS[1],UCS[2],UCS[3],UCS[4],UCS[5],UCS[6],UCS[7],UCS[8],UCS[9]);
	errNo = EXTRA(0,0).mgi(UCS, MCS,msg);
	printf("MCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", MCS[1],MCS[2],MCS[3],MCS[4],MCS[5],MCS[6],MCS[7],MCS[8],MCS[9]);	
	printf("IK Model: errNo('0' means OK): %d , msg: %s\n", errNo, msg);
	printf("FK Model: Sending:\n");
	printf("MCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", MCS[1],MCS[2],MCS[3],MCS[4],MCS[5],MCS[6],MCS[7],MCS[8],MCS[9]);	
	errNo = EXTRA(0,0).mgd(MCS, UCS,msg);
	printf("UCS: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", UCS[1],UCS[2],UCS[3],UCS[4],UCS[5],UCS[6],UCS[7],UCS[8],UCS[9]);
	printf("FK Model: errNo('0' means OK): %d , msg: %s\n", errNo, msg);
INITIALIZE_END

STEP_BEGIN
	//pass PARAMeters to Prigo GM.
	EXTRA(0,0).offsetR1 = PARAM("offsetR1",0);
	EXTRA(0,0).offsetR2 = PARAM("offsetR2",0);
	EXTRA(0,0).offsetR3 = PARAM("offsetR3",0);
	EXTRA(0,0).offsetR4 = PARAM("offsetR4",0);
	EXTRA(0,0).offsetS1 = PARAM("offsetS1",0);
	EXTRA(0,0).offsetS2 = PARAM("offsetS2",0);
	EXTRA(0,0).offsetS3 = PARAM("offsetS3",0);
	EXTRA(0,0).offsetS4 = PARAM("offsetS4",0);
	EXTRA(0,0).l11 = PARAM("l11",0);
	EXTRA(0,0).l12 = PARAM("l12",0);
	EXTRA(0,0).l13 = PARAM("l13",0);
	EXTRA(0,0).l14 = PARAM("l14",0);
	EXTRA(0,0).l24 = PARAM("l24",0);
	EXTRA(0,0).l31 = PARAM("l31",0);
	EXTRA(0,0).l34 = PARAM("l34",0);
	EXTRA(0,0).l35 = PARAM("l35",0);
	EXTRA(0,0).l41 = PARAM("l41",0);
	EXTRA(0,0).l42 = PARAM("l42",0);
	EXTRA(0,0).l43 = PARAM("l43",0);
	EXTRA(0,0).l44 = PARAM("l44",0);
	EXTRA(0,0).l51 = PARAM("l51",0);
	EXTRA(0,0).l52 = PARAM("l52",0);
	EXTRA(0,0).l53 = PARAM("l53",0);
	EXTRA(0,0).l54 = PARAM("l54",0);
	EXTRA(0,0).l55 = PARAM("l55",0);
	EXTRA(0,0).l56 = PARAM("l56",0);
	EXTRA(0,0).a5  = PARAM("a5",0);

	// INPUTs
	double I_ENABLED = INPUT(0,0);
	double I_SHx     = INPUT(1,0);
	double I_SHy     = INPUT(2,0);
	double I_SHz     = INPUT(3,0);
	double I_CHI     = INPUT(4,0);
	double I_PHI     = INPUT(5,0);
	double I_OMEGA   = INPUT(6,0);
	double I_Ox      = INPUT(7,0);
	double I_Oy      = INPUT(8,0);
	double I_Oz      = INPUT(9,0);

	double O_s1         = OUTPUT(0,0);
	double O_s2         = OUTPUT(1,0);
	double O_s3         = OUTPUT(2,0);
	double O_s4         = OUTPUT(3,0);
	double O_phimotor   = OUTPUT(4,0);
	double O_omegamotor = OUTPUT(5,0);

	// =====================================================================================================
	// CALCULATE MODEL (Call prigo.mgi)
	if (I_ENABLED == 1.) {
		double UCS[10] = {0., I_SHx, I_SHy, I_SHz, I_CHI, I_PHI, I_OMEGA, I_Ox, I_Oy, I_Oz};
		double MCS[10] = {0., 0.,0.,0., 0.,0.,0., 0.,0.,300.};
		int err = EXTRA(0,0).mgi(UCS, MCS);	
		if (err){
			err += 100;
			ALARM_WITH_MESSAGE(err, "prigoIK: Kinematic Model error");
		}
		O_s1 = MCS[1];
		O_s2 = MCS[2];
		O_s3 = MCS[3];
		O_s4 = MCS[4];
		O_phimotor = MCS[5];
		O_omegamotor = MCS[6];
	}
	//If module disabled, simply send 0s to the ouput.
	if (I_ENABLED == 0.) {
			O_s1 = 0.;
			O_s2 = 0.;
			O_s3 = 0.;
			O_s4 = 0.;
			O_phimotor = 0.;
			O_omegamotor = 0.;
	}

	// =====================================================================================================
	//Update OUTPUTs
	OUTPUT(0,0)  = O_s1;
	OUTPUT(1,0)  = O_s2;
	OUTPUT(2,0)  = O_s3;
	OUTPUT(3,0)  = O_s4;
	OUTPUT(4,0)  = O_phimotor;
	OUTPUT(5,0)  = O_omegamotor;
STEP_END

FINALIZE_BEGIN
	DEALLOCATE_EXTRA(0);
FINALIZE_END

SAFETY_BEGIN
	//how to behave when a error of the kinematic model transformation happens
	if(ALARM_CODE>100 && ALARM_CODE<200){
		ALARM_SKIP_RESET;
	}
SAFETY_END

