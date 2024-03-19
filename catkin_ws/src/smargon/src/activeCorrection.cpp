// SmarGon MCS2 Active Correction Module
// Wayne Glettig, 30.9.2021
// 
// Upon Loading, this node loads the calibration table csv.
//
// This node then listens for jointstates in SCS (targetSCS) modifies them,
// and outputs the corrected jointstate in SCS (targetSCS_corr)
//
// Listeners:
// std_msgs/Int16 /corr_type                  : correction type	(0: no correction, 1:Sinusoids, 2:LUT 0-order)
// sensor_msgs/JointState /targetSCS          : target position in SCS
//                                              when a message arrives, depending on the 'corr_type',
//                                              the respective algorithm is executed.
// sensor_msgs/JointState /LJUE9_JointState   : get the current Omega position, 
//                                              callback updates OMEGA global variable
//
// Publishers:
// sensor_msgs/JointState /targetSCS_corr     : target position in SCS
// sensor_msgs/JointState /corr_val           : correction SCS
////////////////////////////////////////////////////////////////////////////////

//ROS Includes:
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/JointState.h"

//Other Includes:
#include <cmath>

//Global Variables
int16_t corr_type = 0;
int transition = 1;
sensor_msgs::JointState input_msg;
sensor_msgs::JointState output_msg;
sensor_msgs::JointState corr_msg;
ros::Publisher targetSCS_corr_pub;
ros::Publisher corr_val_pub;
double OMEGA=0;
double OX_corr=0, OY_corr=0, OZ_corr=0;

//LUT Global Variables:
#define LUT_MAX_LINES 10000  
double LUT[LUT_MAX_LINES][6];  //[Line_No] [OMEGA, CHI, PHI, X_corr, Y_corr, Z_corr]
int LUT_nelems = 0;

// Commonly used functions ////////////////////////////////////////////////////
// Load in the Calibration Table (CSV-File)
int load_calibration_table(ros::NodeHandle n) {
    //Get LUT Filename from the ROS Parameter Server
    std::string filename_str;
    n.getParam("active_correction_LUT_file", filename_str);
    char filename[1000];
    strcpy(filename, filename_str.c_str());

    //Define variables
    FILE * pFile;
    char mode[4]="r"; //mode read
    char header_str[1000]; //string buffer
    int lineno = 0;
    //Open file:
    pFile = fopen(filename, mode);
    //Check if file can be opened:
    if (pFile==NULL){ //Error loading file 
        ROS_ERROR("LUT: cannot open file: %s", filename);
        return 1; 
    } else { //Ok, file open
        //Read first line in with the headers
        fscanf (pFile, "%s\n", header_str);
        ROS_INFO("LUT Header: %s", header_str);
        //start reading line by line:
	while (!feof(pFile)) {
            //populate LUT
            fscanf (pFile, "%lf,%lf,%lf,%lf,%lf\n", &LUT[lineno][1], //CHI
                                                    &LUT[lineno][0], //OMEGA
                                                    &LUT[lineno][3], //X_corr
                                                    &LUT[lineno][4], //Y_corr
                                                    &LUT[lineno][5]);//Z_corr
            ROS_INFO("LUT[%05d]: %lf,%lf,%lf,%lf,%lf", lineno,
                                                     LUT[lineno][1],
                                                     LUT[lineno][0],
                                                     LUT[lineno][3],
                                                     LUT[lineno][4],
                                                     LUT[lineno][5]);
    	    lineno++;
    	    if (lineno >= LUT_MAX_LINES) { //if the file is too large for the internal storage
                ROS_ERROR("LUT file has too many lines! LUT_MAX_LINES is %d.", LUT_MAX_LINES);
    	        fclose(pFile);
    	        return 1;
    	    }
        }
        LUT_nelems = lineno; //update global variable how many lines the LUT has.
        ROS_INFO("LUT: %d lines read.", LUT_nelems);
        fclose(pFile);
    }
    return 0;
}

// Extend CHI, so that CHI LUT values cover the whole workspace
int extend_chi() {
    // find largest and smalles values of CHI
    double largestCHI  = -720;
    double smallestCHI = +720;
    for (int i=0; i<LUT_nelems; i++){
       if (LUT[i][1] > largestCHI )  largestCHI  = LUT[i][1]; //scan to find largest CHI
       if (LUT[i][1] < smallestCHI)  smallestCHI = LUT[i][1]; //scan to find smallest CHI
    }
    // If largest and smallest values of CHI don't touch the edge, extend them.
    // Why 95deg? That's because of the boundary condition at 90deg, we want to be sure that at
    // CHI=90deg there are still 4 points around any given input point.
    bool extend_to_95=false, extend_to_0=false;
    if (largestCHI < 95) extend_to_95 = true; //earmark extension to 95deg
    if (smallestCHI > 0) extend_to_0 = true;  //earmark extension to 0deg

    // go through the LUT again and :
    // duplicate entries with largest  CHI values but change CHI to 95deg
    // duplicate entries with smallest CHI values but change CHI to 0deg
    int LUT_nelems_before_extending; //need to remember the size of the LUT before extending.
    LUT_nelems_before_extending = LUT_nelems; //do the loop only these amount of times.
    for (int i=0; i<LUT_nelems_before_extending; i++) {
        if (extend_to_95 && (LUT[i][1] == largestCHI)) { //extend only if earmarked
           //check first if we have space in the LUT:
           if (LUT_nelems + 1 >= LUT_MAX_LINES) {
               ROS_ERROR("LUT: During Omega Extending: No more space in LUT.");
               return 1;
           }
           //Extend to 95deg:
           LUT[LUT_nelems][0] = LUT[i][0];
           LUT[LUT_nelems][1] = 95.0;
           LUT[LUT_nelems][2] = LUT[i][2];
           LUT[LUT_nelems][3] = LUT[i][3];
           LUT[LUT_nelems][4] = LUT[i][4];
           LUT[LUT_nelems][5] = LUT[i][5];
           LUT_nelems++;
        }
        if (extend_to_0 && (LUT[i][1] == smallestCHI)) { //extend only if earmarked
           //check first if we have space in the LUT:
           if (LUT_nelems + 1 >= LUT_MAX_LINES) {
               ROS_ERROR("LUT: During Omega Extending: No more space in LUT.");
               return 1;
           }
           //Extend to 0deg:
           LUT[LUT_nelems][0] = LUT[i][0];
           LUT[LUT_nelems][1] = 0.;
           LUT[LUT_nelems][2] = LUT[i][2];
           LUT[LUT_nelems][3] = LUT[i][3];
           LUT[LUT_nelems][4] = LUT[i][4];
           LUT[LUT_nelems][5] = LUT[i][5];
           LUT_nelems++;
       }
    }
    ROS_INFO("LUT: Extended to %d, added %d elems. (CHI Edge-Extend)", LUT_nelems, LUT_nelems-LUT_nelems_before_extending);
    return 0;
}


// Extend OMEGA, so that OMEGA LUT values cover the whole workspace. 
// Wrap around 360deg to ensure continuity
int extend_omega() {
    // find largest and smalles values of OMEGA
    double largestOMEGA  = -720;
    double smallestOMEGA = +720;
    for (int i=0; i<LUT_nelems; i++){
       if (LUT[i][0] > largestOMEGA)  largestOMEGA  = LUT[i][0]; //scan to find largest OMEGA
       if (LUT[i][0] < smallestOMEGA) smallestOMEGA = LUT[i][0]; //scan to find smallest OMEGA
    }
    // go through the LUT again and :
    // duplicate entries with largest  omega values but change OMEGA to OMEGA-360deg
    // duplicate entries with smallest omega values but change OMEGA to OMEGA+360deg
    int LUT_nelems_before_extending; //need to remember the size of the LUT before extending.
    LUT_nelems_before_extending = LUT_nelems; //do the loop only these amount of times.
    for (int i=0; i<LUT_nelems_before_extending; i++) {
        if (LUT[i][0] == largestOMEGA) { //Do this with the LUT entries with the largest OMEGAs
           //check first if we have space in the LUT:
           if (LUT_nelems + 1 >= LUT_MAX_LINES) {
               ROS_ERROR("LUT: During Omega Extending: No more space in LUT.");
               return 1;
           }
           LUT[LUT_nelems][0] = LUT[i][0] - 360;
           LUT[LUT_nelems][1] = LUT[i][1];
           LUT[LUT_nelems][2] = LUT[i][2];
           LUT[LUT_nelems][3] = LUT[i][3];
           LUT[LUT_nelems][4] = LUT[i][4];
           LUT[LUT_nelems][5] = LUT[i][5];
           LUT_nelems++;
        }
        if (LUT[i][0] == smallestOMEGA) { //Do this with the LUT entries with the smallest OMEGAs
           //check first if we have space in the LUT:
           if (LUT_nelems + 1 >= LUT_MAX_LINES) {
               ROS_ERROR("LUT: During Omega Extending: No more space in LUT.");
               return 1;
           }
           LUT[LUT_nelems][0] = LUT[i][0] + 360;
           LUT[LUT_nelems][1] = LUT[i][1];
           LUT[LUT_nelems][2] = LUT[i][2];
           LUT[LUT_nelems][3] = LUT[i][3];
           LUT[LUT_nelems][4] = LUT[i][4];
           LUT[LUT_nelems][5] = LUT[i][5];
           LUT_nelems++;
       }
    }
    ROS_INFO("LUT: Extended to %d, added %d elems. (OMEGA Wrap-Over)", LUT_nelems, LUT_nelems-LUT_nelems_before_extending);
    return 0;
}


int corr_type_1 (void)  {
    //CORR TYPE 1 IS BILINEAR INTERPOLATION over CHI & OMEGA

    //Get CHI values from the input message
    double CHI;
    CHI   = input_msg.position[4];
    
    //Bound OMEGA to [0,360] and CHI to [0,90]
    OMEGA = fmod(OMEGA, 360); //modulo results are always between 0 and 360. 
                              //In C, the % operator only works on int, therefore
                              //we need to use the fmod function in <cmath>
    if (CHI > 90) CHI = 90;
    if (CHI < 0) CHI = 0;

 
    // For the Bilinear Interpollation, we need the four Edge points (Q11, Q12, Q21, Q22)
    // which are the closest points in the LUT
    int iQ11=-1, iQ12=-1, iQ21=-1, iQ22=-1;

    //Run through LUT and find closest four corner Points Q11, Q12, Q21, Q22:
    for (int i=0; i<LUT_nelems; i++){
       if (LUT[i][0]<=OMEGA && LUT[i][1]<=CHI) { //Q11: [0]<OMEGA && [1]<CHI
               if (iQ11==-1) iQ11=i;
               double dist_i, dist_iQ11;
               dist_i    = (OMEGA-LUT[i]   [0])*(OMEGA-LUT[i]   [0]) + (CHI-LUT[i]   [1])*(CHI-LUT[i]   [1]);
               dist_iQ11 = (OMEGA-LUT[iQ11][0])*(OMEGA-LUT[iQ11][0]) + (CHI-LUT[iQ11][1])*(CHI-LUT[iQ11][1]);
               if (dist_i <= dist_iQ11) iQ11 = i; //Set this point as the new Q11
       }
       if (LUT[i][0]<=OMEGA && LUT[i][1]>CHI) { //Q12: [0]<OMEGA && [1]>CHI
               if (iQ12==-1) iQ12=i;
               double dist_i, dist_iQ12;
               dist_i    = (OMEGA-LUT[i]   [0])*(OMEGA-LUT[i]   [0]) + (CHI-LUT[i]   [1])*(CHI-LUT[i]   [1]);
               dist_iQ12 = (OMEGA-LUT[iQ12][0])*(OMEGA-LUT[iQ12][0]) + (CHI-LUT[iQ12][1])*(CHI-LUT[iQ12][1]);
               if (dist_i <= dist_iQ12) iQ12 = i; //Set this point as the new Q12
       }
       if (LUT[i][0]>OMEGA && LUT[i][1]<=CHI) { //Q21: [0]>OMEGA && [1]<CHI
               if (iQ21==-1) iQ21=i;
               double dist_i, dist_iQ21;
               dist_i    = (OMEGA-LUT[i]   [0])*(OMEGA-LUT[i]   [0]) + (CHI-LUT[i]   [1])*(CHI-LUT[i]   [1]);
               dist_iQ21 = (OMEGA-LUT[iQ21][0])*(OMEGA-LUT[iQ21][0]) + (CHI-LUT[iQ21][1])*(CHI-LUT[iQ21][1]);
               if (dist_i <= dist_iQ21) iQ21 = i; //Set this point as the new Q21
       }
       if (LUT[i][0]>OMEGA && LUT[i][1]>CHI) { //Q22: [0]>OMEGA && [1]>CHI
               if (iQ22==-1) iQ22=i;
               double dist_i, dist_iQ22;
               dist_i    = (OMEGA-LUT[i]   [0])*(OMEGA-LUT[i]   [0]) + (CHI-LUT[i]   [1])*(CHI-LUT[i]   [1]);
               dist_iQ22 = (OMEGA-LUT[iQ22][0])*(OMEGA-LUT[iQ22][0]) + (CHI-LUT[iQ22][1])*(CHI-LUT[iQ22][1]);
               if (dist_i <= dist_iQ22) iQ22 = i; //Set this point as the new Q22
       }
    }

    // Bilinear Interpolation within Q11, Q12, Q21, Q22.
    // let's use x & y for shorter formulas
    double x, x1, x2, y, y1, y2;
    double fQ11_BX, fQ21_BX, fQ12_BX, fQ22_BX;
    double fQ11_BY, fQ21_BY, fQ12_BY, fQ22_BY;
    double fQ11_BZ, fQ21_BZ, fQ12_BZ, fQ22_BZ;
    double BX_corr, BY_corr, BZ_corr;
    // Gather data:
    x=OMEGA;
    y=CHI;
    x1=LUT[iQ11][0];
    x2=LUT[iQ21][0];
    y1=LUT[iQ11][1];
    y2=LUT[iQ12][1];
    fQ11_BX = LUT[iQ11][3]; fQ11_BY = LUT[iQ11][4]; fQ11_BZ = LUT[iQ11][5];
    fQ12_BX = LUT[iQ12][3]; fQ12_BY = LUT[iQ12][4]; fQ12_BZ = LUT[iQ12][5];
    fQ21_BX = LUT[iQ21][3]; fQ21_BY = LUT[iQ21][4]; fQ21_BZ = LUT[iQ21][5];
    fQ22_BX = LUT[iQ22][3]; fQ22_BY = LUT[iQ22][4]; fQ22_BZ = LUT[iQ22][5];

    // Calculate:
    BX_corr = (fQ11_BX*(x2-x)*(y2-y) + fQ21_BX*(x-x1)*(y2-y) + fQ12_BX*(x2-x)*(y-y1) + fQ22_BX*(x-x1)*(y-y1))
              /((x2-x1)*(y2-y1));
    BY_corr = (fQ11_BY*(x2-x)*(y2-y) + fQ21_BY*(x-x1)*(y2-y) + fQ12_BY*(x2-x)*(y-y1) + fQ22_BY*(x-x1)*(y-y1))
              /((x2-x1)*(y2-y1));
    BZ_corr = (fQ11_BZ*(x2-x)*(y2-y) + fQ21_BZ*(x-x1)*(y2-y) + fQ12_BZ*(x2-x)*(y-y1) + fQ22_BZ*(x-x1)*(y-y1))
              /((x2-x1)*(y2-y1));

    //Rotate corr around -OMEGA to find OX, OY, OZ values
    double omega;
    omega = OMEGA/180*M_PI;
    OX_corr = BY_corr*cos(omega) - BZ_corr*sin(omega);
    OY_corr = BY_corr*sin(omega) + BZ_corr*cos(omega);
    OZ_corr = BX_corr;


    //we assume the JointState msg object has be resized to the right size 
    //and we can just update the positions.
    output_msg = input_msg;
	
    output_msg.position[0] = input_msg.position[0]; //SHX
    output_msg.position[1] = input_msg.position[1]; //SHY
    output_msg.position[2] = input_msg.position[2]; //SHZ
    output_msg.position[3] = input_msg.position[3]; //OMEGA
    output_msg.position[4] = input_msg.position[4]; //CHI
    output_msg.position[5] = input_msg.position[5]; //PHI
    output_msg.position[6] = input_msg.position[6]-OX_corr; //OX
    output_msg.position[7] = input_msg.position[7]-OY_corr; //OY
    output_msg.position[8] = input_msg.position[8]-OZ_corr; //OZ

    //Prepare corr message to see/be able to monitor the active correction value
    corr_msg.position[0] = x1; //OMEGA_1
    corr_msg.position[1] = x2; //OMEGA_2
    corr_msg.position[2] = y1; //CHI_1
    corr_msg.position[3] = y2; //CHI_2
    corr_msg.position[4] = 0.0; //
    corr_msg.position[5] = 0.0; //
    corr_msg.position[6] = OX_corr; //OX
    corr_msg.position[7] = OY_corr; //OY
    corr_msg.position[8] = OZ_corr; //OZ

    return 0;
}



// read positions from MCS2 and publish a JointState message on /readback topic
int corr_type_2 (void)  {
    //Let's just copy te input JointState msg object to an output object, 
    //and we can just update the positions.
    output_msg = input_msg;
    double BX_corr=0, BY_corr=0, BZ_corr=0;
    double BX_corr_CHI=0, BY_corr_CHI=0, BZ_corr_CHI=0;

    //Find OMEGA Correction in LUT:
    int nelems = 36;
    double LUT[36][4] = {
        //OMEGA         X         Y         Z
        {  5.0,  0.000009, -0.000061,  0.000111},
        { 15.0,  0.000067,  0.000022, -0.000416},
        { 25.0,  0.000118, -0.000095, -0.001167},
        { 35.0,  0.000173, -0.000291, -0.001988},
        { 45.0,  0.000226, -0.000430, -0.002671},
        { 55.0,  0.000343, -0.000811, -0.003088},
        { 65.0,  0.000463, -0.001516, -0.003128},
        { 75.0,  0.000644, -0.002298, -0.003085},
        { 85.0,  0.000809, -0.003051, -0.003018},
        { 95.0,  0.001033, -0.003640, -0.002840},
        {105.0,  0.001219, -0.003925, -0.002346},
        {115.0,  0.001332, -0.003879, -0.001595},
        {125.0,  0.001511, -0.003745, -0.000746},
        {135.0,  0.001662, -0.003592, -0.000006},
        {145.0,  0.001799, -0.003311,  0.000357},
        {155.0,  0.001872, -0.002769,  0.000452},
        {165.0,  0.001921, -0.002027,  0.000344},
        {175.0,  0.001931, -0.001269,  0.000261},
        {185.0,  0.001920, -0.000811,  0.000035},
        {195.0,  0.001912, -0.000704, -0.000327},
        {205.0,  0.001809, -0.000851, -0.000883},
        {215.0,  0.001736, -0.001090, -0.001574},
        {225.0,  0.001594, -0.001287, -0.002130},
        {235.0,  0.001495, -0.001550, -0.002326},
        {245.0,  0.001438, -0.002214, -0.002288},
        {255.0,  0.001238, -0.002806, -0.002028},
        {265.0,  0.001078, -0.003276, -0.001754},
        {275.0,  0.000953, -0.003524, -0.001459},
        {285.0,  0.000701, -0.003465, -0.000879},
        {295.0,  0.000538, -0.003203, -0.000193},
        {305.0,  0.000421, -0.002868,  0.000555},
        {315.0,  0.000277, -0.002430,  0.001073},
        {325.0,  0.000180, -0.002064,  0.001175},
        {335.0,  0.000127, -0.001553,  0.001069},
        {345.0,  0.000037, -0.000827,  0.000821},
        {355.0, -0.000028,  0.000013,  0.000235}
    };
    // Do linear interpolation based on LUT 	
    double OMEGA_0, OMEGA_1, BX_0, BX_1, BY_0, BY_1, BZ_0, BZ_1;
    for (int i=0; i<nelems-1; i++){ //for OMEGA values between 5 & 355 deg
        if ((OMEGA >= LUT[i][0]) && (OMEGA < LUT[i+1][0])) {
            OMEGA_0 = LUT[i][0]; OMEGA_1 = LUT[i+1][0]; 
            BX_0 = LUT[i][1]; BX_1 = LUT[i+1][1];
            BY_0 = LUT[i][2]; BY_1 = LUT[i+1][2];
            BZ_0 = LUT[i][3]; BZ_1 = LUT[i+1][3];
        }
    }
    if (OMEGA < LUT[0][0]) {  //for OMEGA values below 5deg
            OMEGA_0 = LUT[nelems-1][0]-360; OMEGA_1 = LUT[0][0]; 
            BX_0 = LUT[nelems-1][1]; BX_1 = LUT[0][1];
            BY_0 = LUT[nelems-1][2]; BY_1 = LUT[0][2];
            BZ_0 = LUT[nelems-1][3]; BZ_1 = LUT[0][3];
    }
    if (OMEGA > LUT[nelems-1][0]) {  //for OMEGA values above 355deg
            OMEGA_0 = LUT[nelems-1][0]; OMEGA_1 = LUT[0][0]+360; 
            BX_0 = LUT[nelems-1][1]; BX_1 = LUT[0][1];
            BY_0 = LUT[nelems-1][2]; BY_1 = LUT[0][2];
            BZ_0 = LUT[nelems-1][3]; BZ_1 = LUT[0][3];

    }
    BX_corr = -(BX_0 +(OMEGA-OMEGA_0)/(OMEGA_1-OMEGA_0)*(BX_1-BX_0));// + BX_corr_CHI;
    BY_corr = -(BY_0 +(OMEGA-OMEGA_0)/(OMEGA_1-OMEGA_0)*(BY_1-BY_0));// + BY_corr_CHI;
    BZ_corr = -(BZ_0 +(OMEGA-OMEGA_0)/(OMEGA_1-OMEGA_0)*(BZ_1-BZ_0));//+CHI/90*0.25 + BZ_corr_CHI;
 
    //Rotate corr around -OMEGA to find OX, OY, OZ values
    double omega;
    omega = OMEGA/180*M_PI;
    OX_corr = BY_corr*cos(omega) - BZ_corr*sin(omega);
    OY_corr = BY_corr*sin(omega) + BZ_corr*cos(omega);
    OZ_corr = BX_corr;

    //ROS_INFO("CORR: %lf %lf %lf", OX_corr, OY_corr, OZ_corr);
    
    //Correct values of OX, OY, OZ
    output_msg.position[6] = input_msg.position[6]+OX_corr; //OX
    output_msg.position[7] = input_msg.position[7]+OY_corr; //OY
    output_msg.position[8] = input_msg.position[8]+OZ_corr; //OZ

    //Prepare corr message to see/be able to monitor the active correction value
    corr_msg.position[0] = 0.0; //SHX
    corr_msg.position[1] = 0.0; //SHY
    corr_msg.position[2] = 0.0; //SHZ
    corr_msg.position[3] = 0.0; //OMEGA
    corr_msg.position[4] = 0.0; //CHI
    corr_msg.position[5] = 0.0; //PHI
    corr_msg.position[6] = OX_corr; //OX
    corr_msg.position[7] = OY_corr; //OY
    corr_msg.position[8] = OZ_corr; //OZ


    return 0;
}



int corr_type_3 (void)  { //Hold the correction position
    //we assume the JointState msg object has be resized to the right size 
    //and we can just update the positions.
    output_msg = input_msg;
	
    output_msg.position[0] = input_msg.position[0]; //SHX
    output_msg.position[1] = input_msg.position[1]; //SHY
    output_msg.position[2] = input_msg.position[2]; //SHZ
    output_msg.position[3] = input_msg.position[3]; //OMEGA
    output_msg.position[4] = input_msg.position[4]; //CHI
    output_msg.position[5] = input_msg.position[5]; //PHI
    output_msg.position[6] = input_msg.position[6]+OX_corr; //OX
    output_msg.position[7] = input_msg.position[7]+OY_corr; //OY
    output_msg.position[8] = input_msg.position[8]+OZ_corr; //OZ

    //Prepare corr message to see/be able to monitor the active correction value
    corr_msg.position[0] = 0.0; //SHX
    corr_msg.position[1] = 0.0; //SHY
    corr_msg.position[2] = 0.0; //SHZ
    corr_msg.position[3] = 0.0; //OMEGA
    corr_msg.position[4] = 0.0; //CHI
    corr_msg.position[5] = 0.0; //PHI
    corr_msg.position[6] = OX_corr; //OX
    corr_msg.position[7] = OY_corr; //OY
    corr_msg.position[8] = OZ_corr; //OZ

    return 0;
}

// Subscriber Callback functions///////////////////////////////////////////////
// only do the light stuff here in the callback, 
// setting variables, checking input validity
// do all the device communication and error handling in the main loop. 
void corr_typeCallback(const std_msgs::Int16& msg)
{
    corr_type = msg.data;
    transition = 1;
    return;
}

void readbackOMEGACallback(const sensor_msgs::JointState::ConstPtr& msg) 
{ 
    //Get OMEGA readback value from message 
    OMEGA = msg->position[0]; 
    return; 
}

void targetSCSCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // When a /targetSCS message arrives, calculate correction and output 
    // a /targetSCS_corr message, depending on the corr_type.
    input_msg = *msg;

    if (corr_type == 0) {
        if (transition) {
            ROS_INFO("Active Correction: OFF");
            transition = 0;
        }
        output_msg = input_msg;
        targetSCS_corr_pub.publish(output_msg); //publish output message
        //Prepare corr message to see/be able to monitor the active correction value
        corr_msg.position[0] = 0.0; //SHX
        corr_msg.position[1] = 0.0; //SHY
        corr_msg.position[2] = 0.0; //SHZ
        corr_msg.position[3] = 0.0; //OMEGA
        corr_msg.position[4] = 0.0; //CHI
        corr_msg.position[5] = 0.0; //PHI
        corr_msg.position[6] = 0.0; //OX
        corr_msg.position[7] = 0.0; //OY
        corr_msg.position[8] = 0.0; //OZ
        corr_val_pub.publish(corr_msg);         //publish corr value  message
    } else if (corr_type == 1){
        if (transition) {
            ROS_INFO("Active Correction: ON (Bilinear Interpolation)");
            transition = 0;
        }
        corr_type_1();             //process input_msg to fill out ouput_msp
        targetSCS_corr_pub.publish(output_msg); //publish output message
        corr_val_pub.publish(corr_msg);         //publish corr value  message
    } else if (corr_type == 2){
        if (transition) {
            ROS_INFO("Active Correction: corr_type = 2");
            transition = 0;
        }
        corr_type_2();             //process input_msg to fill out ouput_msp
        targetSCS_corr_pub.publish(output_msg); //publish output message
        corr_val_pub.publish(corr_msg);         //publish corr value  message
    }

    return;
}


// Main ///////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    
    // Set Up ROS: /////////////////////////////////////////////////////////////
    // You must call one of the versions of ros::init() before using any other
    // part of the ROS system. It uses argc and argv.
    ros::init(argc, argv, "activeCorrection");
    // NodeHandle is the main access point to communications with the ROS 
    // system. The first NodeHandle constructed will fully initialize this node,
    // and the last NodeHandle destructed will close down the node.
    ros::NodeHandle n;

    // Load LUT in from CSV file:
    load_calibration_table(n);
    extend_chi();
    extend_omega();

    // Advertise Publishers and Subscribers:
    ros::Subscriber targetSCS_sub = n.subscribe("targetSCS",1000, targetSCSCallback);
    ros::Subscriber LJUE9_OMEGA_sub = n.subscribe("LJUE9_JointState",1000, readbackOMEGACallback);
    ros::Subscriber corr_type_sub = n.subscribe("corr_type",1000, corr_typeCallback);
    targetSCS_corr_pub = n.advertise<sensor_msgs::JointState>("targetSCS_corr", 1000);
    corr_val_pub = n.advertise<sensor_msgs::JointState>("corr_val", 1000);

    corr_msg.header.stamp = ros::Time::now();
    corr_msg.name.resize(9);
    corr_msg.name[0] = "OMEGA_1";
    corr_msg.name[1] = "OMEGA_2";
    corr_msg.name[2] = "CHI_1";
    corr_msg.name[3] = "CHI_2";
    corr_msg.name[4] = "";
    corr_msg.name[5] = "";
    corr_msg.name[6] = "OX_corr";
    corr_msg.name[7] = "OY_corr";
    corr_msg.name[8] = "OZ_corr";

    corr_msg.position.resize(9);

    // ROS Spin.
    ros::spin();

    return 0;
}

