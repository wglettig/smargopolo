/***********************************************************************
 *
 * File name: SmarActMCS.cpp
 * Author   : Wayne Glettig, Marco Salathe
 * Version  : Dez. 2011
 *
 * Modul to communicate with the SmarAct MCS. It sends orders for the
 * respective movement and recives the current positions and states
 *
 **********************************************************************/

#define REALTIME  // use this if RTAI serial port access is required (is used in SmarActMCS_lib.h)

#include <DLCInterface.hpp>
#include <cmath>
#include <cstdio>
#include "prigoStateClass.h"
#include "SmarActMCS_lib.h"


using namespace std;

INITIALIZE_BEGIN
	// create SmarAct MCS Object:
	ALLOCATE_EXTRA(0, 1, SmarActMCS);

	// create SmarAct MCS Object:
	ALLOCATE_EXTRA(1, 1, prigoStateClass *);

	// Initialize STATUS variables:
	printf("INITIALIZATION\n");
	STATUS(0, 0) = 0.0;
	STATUS(1, 0) = 0.0;
	STATUS(2, 0) = 0.0;

	// Default Status of the axes: Uninitialized;
	STATUS(3, 0) = 0.0;
	STATUS(3, 1) = 0.0;
	STATUS(3, 2) = 0.0;
	STATUS(3, 3) = 0.0;
	STATUS(3, 4) = 0.0;

	//Open Serial Port, it stays open and doesn't have to be opened again
	EXTRA(0,0).openSerialPort(0,115200);

	//disable hand control module (shows still sensor data)
	EXTRA(0,0).SHE(2);
INITIALIZE_END

STEP_BEGIN
	// Open Serial port in the first step
//	if (STEP_NUMBER == 1) EXTRA(0,0).openSerialPort(0,115200);

	// Read Inputs:
	int err=0;
	int I_Modus    = (int)INPUT(0, 0);
	int I_Selector = (int)INPUT(1, 0);
	double I_pos[5] = {INPUT(2, 0),
			   INPUT(4, 0), 
			   INPUT(6, 0), 
			   INPUT(8, 0), 
			   INPUT(10, 0)}; 
	double I_v[5] = {INPUT(3, 0),
			 INPUT(5, 0), 
			 INPUT(7, 0), 
			 INPUT(9, 0), 
			 INPUT(11, 0)};

	// connect to prigoStateClass (rename pointer to a readable 'ptPrigoStateClass') 
	EXTRA(1,0) = (prigoStateClass *)(unsigned long)INPUT(12,0);
	prigoStateClass * ptPrigoStateClass = EXTRA(1,0);

	// Read Status:
	int S_subStep =               (int)STATUS(1, 0);  // Sub Step Counter
	int S_currentaxis =           (int)STATUS(2, 0);  // currentaxis
	double S_axisInitOK[5] =     {(int)STATUS(3, 0),  // status to see if axis0 is initialized
	                              (int)STATUS(3, 1),  // status to see if axis1 is initialized
	                              (int)STATUS(3, 2),  // status to see if axis2 is initialized
	                              (int)STATUS(3, 3),  // status to see if axis3 is initialized
	                              (int)STATUS(3, 4)}; // status to see if axis4 is initialized
	double S_sensorfeedback[5] = {(int)STATUS(4, 0),  // status to hold axis0 sensor feedback
	                              (int)STATUS(4, 1),  // status to hold axis1 sensor feedback
	                              (int)STATUS(4, 2),  // status to hold axis2 sensor feedback
	                              (int)STATUS(4, 3),  // status to hold axis3 sensor feedback
	                              (int)STATUS(4, 4)}; // status to hold axis4 sensor feedback
	double S_axisStatus[5] =     {(int)STATUS(5, 0),  // status to hold axis0 status (MCS)
	                              (int)STATUS(5, 1),  // status to hold axis1 status (MCS)
	                              (int)STATUS(5, 2),  // status to hold axis2 status (MCS)
	                              (int)STATUS(5, 3),  // status to hold axis3 status (MCS)
	                              (int)STATUS(5, 4)}; // status to hold axis4 status (MCS)

	// Read PARAMs:
	int P_MCS_connected = (int)PARAM("MCS_connected", 0);

	// On Modus Change set subStep to 0.
	int S_lastModus = (int)STATUS(0, 0);
	if (I_Modus != S_lastModus) {
		S_subStep = 0;
	}

	// Modus Switch
	switch(I_Modus) {
		/***********************************************************************
		 * Mode 1: READY -> Move positioners corresponding to SOLL positions 
		 **********************************************************************/
		case 1: 
			if (S_subStep < 10) {
				S_subStep++;
			} else {
				if(I_Selector<5){
					// moving the stages
					int pos,v;
					pos = (int)(I_pos[I_Selector]*1000000);
					v = (int)(fabs(I_v[I_Selector])*1000000);
					// an attempt for safety:
					ptPrigoStateClass->setCurrentWrite(I_Selector, pos, v, STEP_NUMBER);
					if (ptPrigoStateClass->chkOkToWrite(I_Selector)) {
						if (I_Selector == 4.) { 
							// move phimotor and set speed
							err=EXTRA(0,0).SCLS(I_Selector, v);
							if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
							err=EXTRA(0,0).MAA(I_Selector, pos, 60000);
							if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						} else { 
							// move sliders and set speed
							err=EXTRA(0,0).SCLS(I_Selector, v);
							if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
							err=EXTRA(0,0).MPA(I_Selector, pos, 60000);
							if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						}
					}
				} else {
					// updating values
					if (S_subStep == 10){
						// Send request for current position
						err=EXTRA(0,0).sendGP(0);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						err=EXTRA(0,0).sendGP(1);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						err=EXTRA(0,0).sendGP(2);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						err=EXTRA(0,0).sendGP(3);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						err=EXTRA(0,0).sendGA(4);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						S_subStep=11.;
					} else {
						// read current position and update prigoStateClass
						double position=0;
						err=EXTRA(0,0).setTimestamp(STEP_NUMBER);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						err=EXTRA(0,0).readGP(0.,&position);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						ptPrigoStateClass->sets1(position/1000000.);
						S_sensorfeedback[0]=position;
						err=EXTRA(0,0).readGP(1.,&position);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						ptPrigoStateClass->sets2(position/1000000.);
						S_sensorfeedback[1]=position;
						err=EXTRA(0,0).readGP(2.,&position);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						ptPrigoStateClass->sets3(position/1000000.);
						S_sensorfeedback[2]=position;
						err=EXTRA(0,0).readGP(3.,&position);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						ptPrigoStateClass->sets4(position/1000000.);
						S_sensorfeedback[3]=position;
						err=EXTRA(0,0).readGA(4.,&position);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						ptPrigoStateClass->setphimotor(position/1000000.);
						S_sensorfeedback[4]=position;
						S_subStep=10.;
					}
				}
			}
		break;
		/***********************************************************************
		 * Mode 2 and 3: INITIALIZING
		 *    2: FRM (Find Reference Mark in POS direction)
		 *    3: FRM (Find Reference Mark in NEG direction)
		 **********************************************************************/
		case 2: 
		case 3: 
			int dir;
			dir = I_Modus-2; // Modus 2 -> dir=0(POS) ; Modus 3 -> dir=1(NEG);
			// First Stop all motion.
			if (S_subStep == 0) {
				err=EXTRA(0,0).S(); // :STOP
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				ptPrigoStateClass->sets1_PPK(0.); // Set all PPK Variables to 0.
				ptPrigoStateClass->sets2_PPK(0.);
				ptPrigoStateClass->sets3_PPK(0.);
				ptPrigoStateClass->sets4_PPK(0.);
				ptPrigoStateClass->setphimotor_PPK(0.);
				S_subStep++;
			} else if (S_subStep == 1) {		// :SCLS0,5000000
				err=EXTRA(0,0).SCLS(0, 5000000);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 2) {		// :SCLS0,5000000
				err=EXTRA(0,0).SCLS(1, 5000000);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 3) {		// :SCLS0,5000000
				err=EXTRA(0,0).SCLS(2, 5000000);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 4) {		// :SCLS0,5000000
				err=EXTRA(0,0).SCLS(3, 5000000);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 5) {		// :SCLS0,5000000
				//err=EXTRA(0,0).SCLS(4, 5000000);
				//if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 6) {
				// then send Command to find reference mark Channel0
				err=EXTRA(0,0).FRM(0,dir,60000,0);	// :FRM0,0,60000,0
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 7) {
				// then send Command to find reference mark Channel1
				err=EXTRA(0,0).FRM(1,dir,60000,0);	// :FRM1,0,60000,0
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 8) {
				// then send Command to find reference mark Channel2
				err=EXTRA(0,0).FRM(2,dir,60000,0);	// :FRM2,0,60000,0
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 9) {
				// then send Command to find reference mark Channel3
				err=EXTRA(0,0).FRM(3,dir,60000,0);	// :FRM3,0,60000,0
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 10) {
				// then send Command to find reference mark Channel4
				//err=EXTRA(0,0).FRM(4,0,60000,0);	// :FRM4,0,60000,0, negative direction always so that phi cable doesn't wind up
				//if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 11) {         
			// now start polling MCS, to see what the axis are doing
				S_currentaxis = 0;
				S_subStep++;
			} else if (S_subStep == 12) {         
				// Send "physical position known?" request
				err=EXTRA(0,0).sendGPPK(S_currentaxis);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				// Send status request
				err=EXTRA(0,0).sendGS(S_currentaxis);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep > 12 && S_subStep < 32) {         
				// then wait a couple of steps (20)...
				S_subStep++;
			} else if (S_subStep == 32) {
				double status;
				err=EXTRA(0,0).setTimestamp(STEP_NUMBER);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				// Get "physical position known" reply
				err=EXTRA(0,0).readGPPK(S_currentaxis,&status);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				// update Axis status flag
				S_axisInitOK[S_currentaxis] = status;
				printf ("Axis %d: PPK %.0f, ",S_currentaxis, status);
				// Get status reply
				err=EXTRA(0,0).readGS(S_currentaxis,&status);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				// update Axis status flag
				S_axisStatus[S_currentaxis] = status;
				printf ("Move status %.0f\n", status);
				S_subStep++;
			} else if (S_subStep == 33) {
				// Select next currentaxis (until 4) (like for(i=0;i<4;i++))
				if (S_currentaxis < 4){ 
					S_currentaxis++;
					S_subStep=12; // Loop back to subStep 12
				} else {
					// if 5th axis (axis4) is reached move on.
					S_subStep++;
				}
			} else if (S_subStep == 34) {
				// check if all axis are initialized
				if (S_axisInitOK[0] == 1 && S_axisStatus[0] == 3 &&
				    S_axisInitOK[1] == 1 && S_axisStatus[1] == 3 &&
				    S_axisInitOK[2] == 1 && S_axisStatus[2] == 3 &&
				    S_axisInitOK[3] == 1 && S_axisStatus[3] == 3) {
				    //S_axisInitOK[4] == 1 && S_axisStatus[4] == 3) {
					// INITIALIZATION SUCCESSFUL - REPORT TO prigoCOMMAND!!!
					ptPrigoStateClass->sets1_PPK(1.);
					ptPrigoStateClass->sets2_PPK(1.);
					ptPrigoStateClass->sets3_PPK(1.);
					ptPrigoStateClass->sets4_PPK(1.);
					ptPrigoStateClass->setphimotor_PPK(1.);
					printf("Found Absolute Reference.\n");
					S_subStep++;
				}
				else if (S_axisInitOK[0] == 1 && S_axisStatus[0] == 0) ALARM_WITH_MESSAGE(50, "SmarActMCS: Axis 1 has Status STOP");
				else if (S_axisInitOK[1] == 1 && S_axisStatus[1] == 0) ALARM_WITH_MESSAGE(50, "SmarActMCS: Axis 2 has Status STOP");
				else if (S_axisInitOK[2] == 1 && S_axisStatus[2] == 0) ALARM_WITH_MESSAGE(50, "SmarActMCS: Axis 3 has Status STOP");
				else if (S_axisInitOK[3] == 1 && S_axisStatus[3] == 0) ALARM_WITH_MESSAGE(50, "SmarActMCS: Axis 4 has Status STOP");
				//else if (S_axisInitOK[4] == 1 && S_axisStatus[4] == 0) ALARM_WITH_MESSAGE(50, "SmarActMCS: Axis 5 has Status STOP");
				else {
					S_subStep=11; // Loop back to subStep 11
				}
			} else if (S_subStep > 34 && S_subStep < 55) {         
				// then wait a couple of steps (20)...
				S_subStep++;
			} else if (S_subStep == 55) {
				// Drive to initial position.
				int pos = (int)(I_pos[0]*1000000);
				err=EXTRA(0,0).SCLS(0, 2000000);  // this is the SPEED to use to drive to initial position
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				err=EXTRA(0,0).MPA(0, pos, 60000);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 56) {
				int pos = (int)(I_pos[1]*1000000);
				err=EXTRA(0,0).SCLS(1, 2000000); // this is the SPEED to use to drive to initial position
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				err=EXTRA(0,0).MPA(1, pos, 60000);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 57) {
				int pos = (int)(I_pos[2]*1000000);
				err=EXTRA(0,0).SCLS(2, 2000000); // this is the SPEED to use to drive to initial position
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				err=EXTRA(0,0).MPA(2, pos, 60000);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 58) {
				int pos = (int)(I_pos[3]*1000000);
				err=EXTRA(0,0).SCLS(3, 2000000); // this is the SPEED to use to drive to initial position
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				err=EXTRA(0,0).MPA(3, pos, 60000);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 59) {
				//int pos = (int)(I_pos[4]*1000000);
				//err=EXTRA(0,0).SCLS(4, 40000000); // this is the SPEED to use to drive to initial position
				//if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				//err=EXTRA(0,0).MAA(4, pos, 60000);
				//if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 60) {
			// request to Read current position
			 	err=EXTRA(0,0).sendGP(0);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 61) {
				err=EXTRA(0,0).sendGP(1);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 62) {
				err=EXTRA(0,0).sendGP(2);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 63) {
				err=EXTRA(0,0).sendGP(3);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 64) {
				err=EXTRA(0,0).sendGA(4);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 65) {
			// Read current position reply
				double position=0;
				err=EXTRA(0,0).setTimestamp(STEP_NUMBER);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				err=EXTRA(0,0).readGP(0.,&position);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				ptPrigoStateClass->sets1(position/1000000.); // update prigoStateClass
				err=EXTRA(0,0).readGP(1.,&position);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				ptPrigoStateClass->sets2(position/1000000.); // update prigoStateClass
				err=EXTRA(0,0).readGP(2.,&position);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				ptPrigoStateClass->sets3(position/1000000.); // update prigoStateClass
				err=EXTRA(0,0).readGP(3.,&position);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				ptPrigoStateClass->sets4(position/1000000.); // update prigoStateClass
				err=EXTRA(0,0).readGA(4.,&position);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				ptPrigoStateClass->setphimotor(position/1000000.); // update prigoStateClass
			// See if position is reached:
				if ((fabs(I_pos[0]-ptPrigoStateClass->s1.value)<0.001) &&
					(fabs(I_pos[1]-ptPrigoStateClass->s2.value)<0.001) &&
					(fabs(I_pos[2]-ptPrigoStateClass->s3.value)<0.001) &&
					(fabs(I_pos[3]-ptPrigoStateClass->s4.value)<0.001) ) S_subStep++;
				else S_subStep = 60;
			} else if (S_subStep == 66) {
			// If position reached, change MODE to 2.
				ptPrigoStateClass->setMODE(2);
				printf("YEAH INITALiZATION SUCCESSFUL\n");
				S_subStep++;
			} else if (S_subStep == 67) {
				// wait
			}
		break;
		/***********************************************************************
		 * Mode 0: UNINIZIALIZED/IDLE/ERROR -> movements disabled 
		 **********************************************************************/
		default: //Modus 0 or other: DISABLE
			// First Stop all motion.
			if (S_subStep < 20) {
				ptPrigoStateClass->sets1_PPK(0.); // Set all PPK Variables to 0.
				ptPrigoStateClass->sets2_PPK(0.);
				ptPrigoStateClass->sets3_PPK(0.);
				ptPrigoStateClass->sets4_PPK(0.);
				ptPrigoStateClass->setphimotor_PPK(0.);
				S_subStep++;
			} else if (S_subStep == 20) {
				err=EXTRA(0,0).SCM(1);
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				err=EXTRA(0,0).clearBuf();
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 21) {
				// then stop motion on all channels
				err=EXTRA(0,0).S(); // :STOP
				if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
				S_subStep++;
			} else if (S_subStep == 22 || S_subStep == 23) {
				// then start polling MCS, to see if MCS is still attached.
				// poll every half second
				double pollperiod = 0.1;
				if (STEP_NUMBER%(int)(pollperiod/PERIOD/2)==0) {
					if (S_subStep == 22) {
						// Send status request
						err=EXTRA(0,0).sendGP(0);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						err=EXTRA(0,0).sendGP(1);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						err=EXTRA(0,0).sendGP(2);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						err=EXTRA(0,0).sendGP(3);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						err=EXTRA(0,0).sendGA(4);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						S_subStep = 23;
					} else {
						// Get Status reply
						double position;
						err=EXTRA(0,0).setTimestamp(STEP_NUMBER);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						err=EXTRA(0,0).readGP(0, &position);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						ptPrigoStateClass->sets1(position/1000000.); // update prigoStateClass
						err=EXTRA(0,0).readGP(1, &position);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						ptPrigoStateClass->sets2(position/1000000.); // update prigoStateClass
						err = EXTRA(0,0).readGP(2, &position);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						ptPrigoStateClass->sets3(position/1000000.); // update prigoStateClass
						err = EXTRA(0,0).readGP(3, &position);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						ptPrigoStateClass->sets4(position/1000000.); // update prigoStateClass
						err = EXTRA(0,0).readGA(4, &position);
						if (err) ALARM_WITH_MESSAGE(err, "SmarActMCS: Connection error");
						ptPrigoStateClass->setphimotor(position/1000000.); // update prigoStateClass
						S_subStep = 22;
					}
				}
			}
		break;
	}

	// STATUS
	STATUS(0, 0) = I_Modus;
	STATUS(1, 0) = S_subStep;
	STATUS(2, 0) = S_currentaxis;
	STATUS(3, 0) = S_axisInitOK[0];
	STATUS(3, 1) = S_axisInitOK[1];
	STATUS(3, 2) = S_axisInitOK[2];
	STATUS(3, 3) = S_axisInitOK[3];
	STATUS(3, 4) = S_axisInitOK[4];
	STATUS(4, 0) = S_sensorfeedback[0];
	STATUS(4, 1) = S_sensorfeedback[1];
	STATUS(4, 2) = S_sensorfeedback[2];
	STATUS(4, 3) = S_sensorfeedback[3];
	STATUS(4, 4) = S_sensorfeedback[4];
	STATUS(5, 0) = S_axisStatus[0];
	STATUS(5, 1) = S_axisStatus[1];
	STATUS(5, 2) = S_axisStatus[2];
	STATUS(5, 3) = S_axisStatus[3];
	STATUS(5, 4) = S_axisStatus[4];

	// PARAMs:
	PARAM("MCS_connected", 0) = P_MCS_connected;

	// OUTPUTs:
	OUTPUT(0, 0) = ptPrigoStateClass->s1.value;
	OUTPUT(1, 0) = ptPrigoStateClass->s2.value;
	OUTPUT(2, 0) = ptPrigoStateClass->s3.value;
	OUTPUT(3, 0) = ptPrigoStateClass->s4.value;
	OUTPUT(4, 0) = ptPrigoStateClass->phimotor.value;
	OUTPUT(5, 0) = PARAM("MCS_connected", 0);
STEP_END

FINALIZE_BEGIN
	// Setting all the close loop speeds to 0 (default), otherwise axis might not move correctly in close loop mode
	for (int i=0; i<5;i++){
		EXTRA(0,0).SCLS(i, 0);
		usleep(10000);
	};
	// enable the hand control module
	EXTRA(0,0).SHE(1);
	usleep(10000);
	// close port
	EXTRA(0,0).closeSerialPort();
	DEALLOCATE_EXTRA(0);
	DEALLOCATE_EXTRA(1);
FINALIZE_END

SAFETY_BEGIN
	//treat here all SmarAct related errors and what actions should be taken
	if (ALARM_CODE > 1 && ALARM_CODE < 100){
		ALARM_RESUME_RESET;
	}
SAFETY_END

