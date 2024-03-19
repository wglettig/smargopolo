// prigo.h: interface for the prigo class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PRIGO_H__13F6686D_FAAF_41E1_8320_FAA4AFD176EE__INCLUDED_)
#define AFX_PRIGO_H__13F6686D_FAAF_41E1_8320_FAA4AFD176EE__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <stdio.h>

class prigo  
{
public:
	double l11, l12, l13, l14;
	double l21, l22, l23, l24;
	double l31, l32, l33, l34, l35;
	double l41, l42, l43, l44;
	double l51, l52, l53, l54, l55, l56;
	double a5; //angle of 4th slider

	double offsetR1, offsetR2, offsetR3, offsetR4; //Slider Offsets (R: Reference, on Base)
	double offsetS1, offsetS2, offsetS3, offsetS4; //Slider Offsets (S: on Slider)

	double s1, s2, s3, s4;     //Slider Motor Coordinates
	double omega_pos, phi_pos; //Omega & Phi Motor Coordinates


	double SHx, SHy, SHz; //Sample Holder Vector
	double Ox, Oy, Oz;    //Sample Centre Position (0 Ref: centre of aerotech tabletop)

	int mgi(double *Point_op, double *Point_art, char *msg);
	int mgi(double *Point_op, double *Point_art);
	int mgd(double* Point_art, double* Point_op, char *msg);
	int mgd(double* Point_art, double* Point_op);
	int readParamFile(char* filename);
	int writeParamFile(char* filename);
	int printParams();

	//CATIA Law File Stuff:
private:
	FILE * CATIALawFile;
	int  CATIALawFile_counter;
public:
	int switchOn_CATIALawFile(char* filename);
	int switchOff_CATIALawFile(void);
	int CATIALawFile_writeLine(double UCS1,double UCS2,double UCS3,double UCS4,double UCS5,double UCS6,double UCS7,double UCS8,double UCS9);


	int logString(char* string);
	void test();
	double getPhiPos();
	double getOmegaPos();
	double getS4();
	double getS3();
	double getS2();
	double getS1();
	void setSHVector(double shx, double shy, double shz);
	void setOVector(double ox, double oy, double oz);
	void setOffsetR1(double offset); //offsets Reference (base)
	void setOffsetR2(double offset);
	void setOffsetR3(double offset);
	void setOffsetR4(double offset);
	void setOffsetS1(double offset); //offsets Sliders
	void setOffsetS2(double offset);
	void setOffsetS3(double offset);
	void setOffsetS4(double offset);
	double getOffsetR1();
	double getOffsetR2();
	double getOffsetR3();
	double getOffsetR4();
	double getOffsetS1();
	double getOffsetS2();
	double getOffsetS3();
	double getOffsetS4();

	prigo();
	virtual ~prigo();
};

#endif // !defined(AFX_PRIGO_H__13F6686D_FAAF_41E1_8320_FAA4AFD176EE__INCLUDED_)
