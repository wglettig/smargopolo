#include <stdio.h>


#include <iostream>
//Chrono can be used to measure the time of certain events in [us]:
#include <chrono>
// auto start = std::chrono::high_resolution_clock::now();
// //do stuff here//
// auto stop = std::chrono::high_resolution_clock::now();
// auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
// std::cout << duration.count() << std::endl;


//Global Variables
double CHI=0, OMEGA=0;

//LUT Global Variables:
#define LUT_MAX_LINES 10000  
double LUT[LUT_MAX_LINES][6];  //[Line_No] [OMEGA, CHI, PHI, X_corr, Y_corr, Z_corr]
int LUT_nelems = 0;

// Commonly used functions ////////////////////////////////////////////////////
int load_calibration_table(void) {
    //Define variables
    char filename[32]= "LUT.csv";
    FILE * pFile;
    char mode[4]="r"; //mode read
    char header_str[1000]; //string buffer
    int lineno = 0;
    //Open file:
    pFile = fopen(filename, mode);
    //Check if file can be opened:
    if (pFile==NULL){ //Error loading file 
        printf("LUT: cannot open file: %s\n", filename);
        return 1;
    } else { //Ok, file open
        //Read first line in with the headers
        fscanf (pFile, "%s\n", header_str);
        printf("LUT Header: %s\n", header_str);
        //start reading line by line:
        while (!feof(pFile)) {
            //populate LUT
            fscanf (pFile, "%lf,%lf,%lf,%lf,%lf\n", &LUT[lineno][1], //CHI
                                                    &LUT[lineno][0], //OMEGA
						    &LUT[lineno][3], //X_corr
						    &LUT[lineno][4], //Y_corr
						    &LUT[lineno][5]);//Z_corr
            printf ("LUT[%05d]: %lf,%lf,%lf,%lf,%lf\n", lineno, 
			                             LUT[lineno][1],  
						     LUT[lineno][0],  
						     LUT[lineno][3],  
						     LUT[lineno][4],  
						     LUT[lineno][5]);
            lineno++;
            if (lineno >= LUT_MAX_LINES) { //if the file is too large for the internal storage
                printf("LUT file has too many lines! LUT_MAX_LINES is %d.\n", LUT_MAX_LINES);
                fclose(pFile);
                return 1;
            }
        }
        LUT_nelems = lineno; //update global variable how many lines the LUT has.
        printf("LUT: %d lines read.\n", LUT_nelems);
        fclose(pFile);
    }
    return 0;
}

int extend_chi() {
    // find largest and smalles values of CHI
    double largestCHI  = -720;
    double smallestCHI = +720;
    for (int i=0; i<LUT_nelems; i++){
       if (LUT[i][1] > largestCHI )  largestCHI  = LUT[i][1]; //scan to find largest CHI
       if (LUT[i][1] < smallestCHI)  smallestCHI = LUT[i][1]; //scan to find smallest CHI
    }
    //if largest and smallest values of CHI don't touch the edge, extend them.
    //Why 95deg? That's because of the boundary condition at 90deg, 
    //and how it find the four corners later on. With 95 we have clear workspace coverage.
    bool extend_to_95=false, extend_to_0=false;
    if (largestCHI < 95) extend_to_95 = true;
    if (smallestCHI > 0) extend_to_0 = true;


    // go through the LUT again and :
    // duplicate entries with largest  CHI values but change CHI to 95deg
    // duplicate entries with smallest CHI values but change CHI to 0deg
    int LUT_nelems_before_extending; //need to remember the size of the LUT before extending.
    LUT_nelems_before_extending =LUT_nelems;
    for (int i=0; i<LUT_nelems_before_extending; i++){
        if (extend_to_95 && (LUT[i][1] == largestCHI)) {
           //check first if we have space in the LUT:
	   if (LUT_nelems + 1 >= LUT_MAX_LINES) {
	       printf("ERROR during Extending, no more space in LUT");
	       return 1;
           }
           LUT[LUT_nelems][0] = LUT[i][0];
           LUT[LUT_nelems][1] = 95.0; 
           LUT[LUT_nelems][2] = LUT[i][2];
           LUT[LUT_nelems][3] = LUT[i][3];
           LUT[LUT_nelems][4] = LUT[i][4];
           LUT[LUT_nelems][5] = LUT[i][5];
	   LUT_nelems++;
        }
        if (extend_to_0 && LUT[i][1] == smallestCHI) {
           //check first if we have space in the LUT:
	   if (LUT_nelems + 1 >= LUT_MAX_LINES) {
	       printf("ERROR during Extending, no more space in LUT");
	       return 1;
           }
           LUT[LUT_nelems][0] = LUT[i][0];
           LUT[LUT_nelems][1] = 0.;
           LUT[LUT_nelems][2] = LUT[i][2];
           LUT[LUT_nelems][3] = LUT[i][3];
           LUT[LUT_nelems][4] = LUT[i][4];
           LUT[LUT_nelems][5] = LUT[i][5];
	   LUT_nelems++;
       }
    }
    printf("LUT: Extended to %d, added %d elems. (CHI Edge-Extend)\n", LUT_nelems, LUT_nelems-LUT_nelems_before_extending);
    return 0;
}
	


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
    LUT_nelems_before_extending =LUT_nelems;
    for (int i=0; i<LUT_nelems_before_extending; i++){
        if (LUT[i][0] == largestOMEGA) {
           //check first if we have space in the LUT:
	   if (LUT_nelems + 1 >= LUT_MAX_LINES) {
	       printf("ERROR during Extending, no more space in LUT");
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
        if (LUT[i][0] == smallestOMEGA) {
           //check first if we have space in the LUT:
	   if (LUT_nelems + 1 >= LUT_MAX_LINES) {
	       printf("ERROR during Extending, no more space in LUT");
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
    printf("LUT: Extended to %d, added %d elems. (OMEGA Wrap-Over)\n", LUT_nelems, LUT_nelems-LUT_nelems_before_extending);
    return 0;
}
	


int corr_type_2 (double OMEGA, double CHI) {

   //Get OMEGA & CHI values from the input message
    //double CHI, OMEGA;
    //OMEGA = input_msg.position[3];
    //CHI   = input_msg.position[4];
    printf("OMEGA: %lf, CHI: %lf\n", OMEGA, CHI);

auto start = std::chrono::high_resolution_clock::now();
    // For the Bilinear Interpollation, we need the four Edge points (Q11, Q12, Q21, Q22)
    // which are the closest points in the LUT
    int iQ11=-1, iQ12=-1, iQ21=-1, iQ22=-1;

    //Run through LUT and find closest four corner Points Q11, Q12, Q21, Q22:
    //
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

auto stop = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
std::cout << "Interpollation took " << duration.count() << "[us]" << std::endl;
    // Print output
    printf ("Q11: LUT[%05d]: OMEGA=%lf, CHI=%lf, X_corr=%lf, Y_corr=%lf, Z_corr=%lf\n",
                                                    iQ11,
                                                    LUT[iQ11][0],
                                                    LUT[iQ11][1],
                                                    LUT[iQ11][3],
                                                    LUT[iQ11][4],
                                                    LUT[iQ11][5]);
    printf ("Q12: LUT[%05d]: OMEGA=%lf, CHI=%lf, X_corr=%lf, Y_corr=%lf, Z_corr=%lf\n",
                                                    iQ12,
                                                    LUT[iQ12][0],
                                                    LUT[iQ12][1],
                                                    LUT[iQ12][3],
                                                    LUT[iQ12][4],
                                                    LUT[iQ12][5]);
    printf ("Q21: LUT[%05d]: OMEGA=%lf, CHI=%lf, X_corr=%lf, Y_corr=%lf, Z_corr=%lf\n",
                                                    iQ21,
                                                    LUT[iQ21][0],
                                                    LUT[iQ21][1],
                                                    LUT[iQ21][3],
                                                    LUT[iQ21][4],
                                                    LUT[iQ21][5]);
    printf ("Q22: LUT[%05d]: OMEGA=%lf, CHI=%lf, X_corr=%lf, Y_corr=%lf, Z_corr=%lf\n",
                                                    iQ22,
                                                    LUT[iQ22][0],
                                                    LUT[iQ22][1],
                                                    LUT[iQ22][3],
                                                    LUT[iQ22][4],
                                                    LUT[iQ22][5]);

    printf ("BX_corr: %lf\n", BX_corr);
    printf ("BY_corr: %lf\n", BY_corr);
    printf ("BZ_corr: %lf\n", BZ_corr);

/*    if (iQ11 == -1 && iQ12 == -1) {
       OMEGA = OMEGA +360;
       for (int i=0; i<LUT_nelems; i++){
           if (LUT[i][0]<OMEGA && LUT[i][1]<CHI) { //Q11: [0]<OMEGA && [1]<CHI
               if (iQ11==-1) iQ11=i;
    	       double dist_i, dist_iQ11;
    	       dist_i    = (OMEGA-LUT[i]   [0])*(OMEGA-LUT[i]   [0]) + (CHI-LUT[i]   [1])*(CHI-LUT[i]   [1]);
    	       dist_iQ11 = (OMEGA-LUT[iQ11][0])*(OMEGA-LUT[iQ11][0]) + (CHI-LUT[iQ11][1])*(CHI-LUT[iQ11][1]);
    	       if (dist_i <= dist_iQ11) iQ11 = i; //Set this point as the new Q11
           }
           if (LUT[i][0]<OMEGA && LUT[i][1]>CHI) { //Q12: [0]<OMEGA && [1]>CHI
               if (iQ12==-1) iQ12=i;
	       double dist_i, dist_iQ12;
	       dist_i    = (OMEGA-LUT[i]   [0])*(OMEGA-LUT[i]   [0]) + (CHI-LUT[i]   [1])*(CHI-LUT[i]   [1]);
	       dist_iQ12 = (OMEGA-LUT[iQ12][0])*(OMEGA-LUT[iQ12][0]) + (CHI-LUT[iQ12][1])*(CHI-LUT[iQ12][1]);
	       if (dist_i <= dist_iQ12) iQ12 = i; //Set this point as the new Q12
	   }
       }

    printf ("Q11: LUT[%05d]: OMEGA=%lf, CHI=%lf, X_corr=%lf, Y_corr=%lf, Z_corr=%lf\n",
                                                    iQ11,
                                                    LUT[iQ11][0],
                                                    LUT[iQ11][1],
                                                    LUT[iQ11][3],
                                                    LUT[iQ11][4],
                                                    LUT[iQ11][5]);
    printf ("Q12: LUT[%05d]: OMEGA=%lf, CHI=%lf, X_corr=%lf, Y_corr=%lf, Z_corr=%lf\n",
                                                    iQ12,
                                                    LUT[iQ12][0],
                                                    LUT[iQ12][1],
                                                    LUT[iQ12][3],
                                                    LUT[iQ12][4],
                                                    LUT[iQ12][5]);
    }
*/
    return 0;

    //
    //
    //
    /* Do linear interpolation based on LUT     
    double CHI_0, CHI_1, BX_0, BX_1, BY_0, BY_1, BZ_0, BZ_1;
    for (int i=0; i<CHI_LUT_nelems-1; i++){ //for CHI values between 1.5 & 88.5 deg
        if ((CHI >= CHI_LUT[i][0]) && (CHI < CHI_LUT[i+1][0])) {
            CHI_0 = CHI_LUT[i][0];   CHI_1 = CHI_LUT[i+1][0];
            BX_0 =  CHI_LUT[i][1];   BX_1 =  CHI_LUT[i+1][1];
            BY_0 =  CHI_LUT[i][2];   BY_1 =  CHI_LUT[i+1][2];
            BZ_0 =  CHI_LUT[i][3];   BZ_1 =  CHI_LUT[i+1][3];
        }
    }
    BX_corr_CHI = -(BX_0 +(CHI-CHI_0)/(CHI_1-CHI_0)*(BX_1-BX_0));
    BY_corr_CHI = -(BY_0 +(CHI-CHI_0)/(CHI_1-CHI_0)*(BY_1-BY_0));
    BZ_corr_CHI = -(BZ_0 +(CHI-CHI_0)/(CHI_1-CHI_0)*(BZ_1-BZ_0));

    if (CHI < CHI_LUT[0][0]) {  //for CHI values below 1.5deg
            BX_corr_CHI = CHI_LUT[0][1];
            BY_corr_CHI = CHI_LUT[0][2];
            BZ_corr_CHI = CHI_LUT[0][3];
    }
    if (CHI > CHI_LUT[CHI_LUT_nelems-1][0]) {  //for CHI values above 88.5deg
            BX_corr_CHI = CHI_LUT[CHI_LUT_nelems-1][1];
            BY_corr_CHI = CHI_LUT[CHI_LUT_nelems-1][2];
            BZ_corr_CHI = CHI_LUT[CHI_LUT_nelems-1][3];

    }
*/

} 



int main( int argc, char *argv[] )  
{
    printf("Loading LUT.");
auto start = std::chrono::high_resolution_clock::now();
    load_calibration_table();
auto stop = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
std::cout << "LUT Reading took " << duration.count() << "[us]" << std::endl;

start = std::chrono::high_resolution_clock::now();
    extend_chi();
stop = std::chrono::high_resolution_clock::now();
duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
std::cout << "CHI extension took " << duration.count() << "[us]" << std::endl;

start = std::chrono::high_resolution_clock::now();
    extend_omega();
duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
stop = std::chrono::high_resolution_clock::now();
std::cout << "OMEGA wraparound took " << duration.count() << "[us]" << std::endl;

    for (int i=0;i<100;i++) {
        for (int j=0;j<100;j++) {
	    double OMEGA, CHI;
	    OMEGA = (360./100.*(double)i);
	    CHI = (90./100.*(double)j);
	    corr_type_2(OMEGA, CHI);
	}
    }
    return 0;
}
