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

    //Motor Coordinates (MCS): running/start/limpos/limneg
    double q1, q2, q3, q4, q5, q6;
    double q1v, q2v, q3v, q4v, q5v, q6v;
    double q1_start , q2_start , q3_start , q4_start , q5_start , q6_start ;
    double q1_limpos, q2_limpos, q3_limpos, q4_limpos, q5_limpos, q6_limpos;
    double q1_limneg, q2_limneg, q3_limneg, q4_limneg, q5_limneg, q6_limneg;

    //SmarGon Coordinates (SCS): running/start/limpos/limneg
    double SHX, SHY, SHZ;
    double OMEGA, CHI, PHI;
    double OX, OY, OZ;
    double SHXv, SHYv, SHZv;
    double OMEGAv, CHIv, PHIv;
    double OXv, OYv, OZv;
    double   SHX_start, SHY_start, SHZ_start;
    double OMEGA_start, CHI_start, PHI_start;
    double    OX_start,  OY_start,  OZ_start;
    double   SHX_limpos, SHY_limpos, SHZ_limpos;
    double OMEGA_limpos, CHI_limpos, PHI_limpos;
    double    OX_limpos,  OY_limpos,  OZ_limpos;
    double   SHX_limneg, SHY_limneg, SHZ_limneg;
    double OMEGA_limneg, CHI_limneg, PHI_limneg;
    double    OX_limneg,  OY_limneg,  OZ_limneg;

    //Beamline Coordinates (BCS): running/start/limpos/limneg
    double BX, BY, BOMEGA, BCHI, BPHI;  
    double BX_start , BY_start , BOMEGA_start , BCHI_start , BPHI_start;
    double BX_limpos, BY_limpos, BOMEGA_limpos, BCHI_limpos, BPHI_limpos;
    double BX_limneg, BY_limneg, BOMEGA_limneg, BCHI_limneg, BPHI_limneg;
    
    //Inverse and Direct Kinematic models:
    int IK(double *SCS, double *SCSv, double *MCS, double *MCSv, char *msg);
    int IK(double *SCS, double *MCS, char *msg);
    int IK(double *SCS, double *MCS);
    int FK(double *MCS, double *MCSv, double *SCS, double *SCSv, char *msg);
    int FK(double *MCS, double *SCS, char *msg);
    int FK(double *MCS, double *SCS);
    int BCStoMCS(double *BCS, double *SCS);
    int runIKv(void);
    int runIK(void);
    int runFK(void);

    //File Functions:
    int readParamFile(char* filename);
    int writeParamFile(char* filename);
    int printParams();
    int logString(char* string);

    //Trajectory Functions:
    double dt;
    double traj_xT[6];
    double traj_x[6];
    double traj_v[6];
    double traj_a[6];
    double traj_vmax[6];
    double traj_amax[6];
    int traj_reset();
    int traj_runstep();


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

