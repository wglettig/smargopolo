// smargon.h: interface for the smargon class.
// Wayne Glettig 13.9.2020
//////////////////////////////////////////////////////////////////////
// Please also consult "SmarGon Geometrical Model (IK,FK).docx"
// (version 17.7.2020) for an explanation of the coordinate systems
// and geometrical lengths.
// 
//
//
//

#include <stdio.h>

class smargon  
{
public:
    //Geometrical Lengths:
    double l01, l11, l12, l21, l22, l23;
    double l31, l32, l33, l34;
    double l41, l42, l51, l52, l61;
    double l71, l72, l73, l74;
    double offsetQ1, offsetQ2, offsetQ3; //Slider/motor offsets
    double offsetQ4, offsetQ5, offsetQ6; 

    //Motor Coordinates (MCS):
    double q1, q2, q3, q4, q5, q6;
 
    //SmarGon Coordinates (SCS):
    double SHX, SHY, SHZ;   //Sample Holder Vector
    double OMEGA, CHI, PHI; //Angles
    double OX, OY, OZ;      //Sample Centre Position 

    //Beamline Coordinates (BCS):
    double BLX, BLY;  //Position of tip on the microscope image for User
    double BLX_raw, BLY_raw;  //Position of tip 
    double BLX_zero, BLY_zero; //Position of zero

    //Inverse and Direct Kinematic models:
    int IK(double *SCS, double *MCS, char *msg);
    int IK(double *SCS, double *MCS);
    int FK(double *MCS, double *SCS, char *msg);
    int FK(double *MCS, double *SCS);
    int BCStoMCS(double *BCS, double *SCS);

    //File Functions:
    int readParamFile(char* filename);
    int writeParamFile(char* filename);
    int printParams();
    int logString(char* string);

    //CATIA Law File Stuff:
private:
    FILE * CATIALawFile;
    int  CATIALawFile_counter;
public:
    int switchOn_CATIALawFile(char* filename);
    int switchOff_CATIALawFile(void);
    int CATIALawFile_writeLine(double UCS1,double UCS2,double UCS3,double UCS4,double UCS5,double UCS6,double UCS7,double UCS8,double UCS9);

    //Constructor/Destructor
    smargon();
    virtual ~smargon();
};

