// SmarGon MCS2 interface for ROS
// Wayne Glettig, 7.12.2020
// 
// This node handles communication with the SmarAct MCS2 and
// provides the following ROS topics interfaces:
// Listeners:
// std_msgs/Int16         /mode         : Operation mode of the smargopolo control system
// sensor_msgs/JointState /targetMCS    : Target position
// Publishers:
// std_msgs/Int16         /modeRequiest
// sensor_msgs/JointState /readbackMCS  : Readback position (read from MCS2)
////////////////////////////////////////////////////////////////////////////////

//ROS Includes:
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

//SmarAct SDK Includes:
//#include <SmarActControl.h>

//Other Includes:
#include <cmath>
#include <sstream>

///Chrono can be used to measure the time of certain events in [us]:
#include <chrono>
// auto start = std::chrono::high_resolution_clock::now();
// //do stuff here//
// auto stop = std::chrono::high_resolution_clock::now();
// auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
// std::cout << duration.count() << std::endl;


//This is to allow printf's of binary numbers:
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

//This is used to format the state flags to print them with printf
#define STATEFLAGS_PATTERN "%c%c%c%c%c %c %c%c %c%c%c%c%c%c%c%c%c%c%c"
#define STATEFLAGS(flag)  \
  (flag & 0x01 ? '1' : '0'), \
  (flag & 0x02 ? '1' : '0'), \
  (flag & 0x04 ? '1' : '0'), \
  (flag & 0x08 ? '1' : '0'), \
  (flag & 0x10 ? '1' : '0'), \
  (flag & 0x20 ? '1' : '0'), \
  (flag & 0x40 ? '1' : '0'), \
  (flag & 0x80 ? '1' : '0'), \
  (flag & 0x100 ? '1' : '0'), \
  (flag & 0x200 ? '1' : '0'), \
  (flag & 0x400 ? '1' : '0'), \
  (flag & 0x800 ? '1' : '0'), \
  (flag & 0x1000 ? '1' : '0'), \
  (flag & 0x2000 ? '1' : '0'), \
  (flag & 0x4000 ? '1' : '0'), \
  (flag & 0x8000 ? '1' : '0'), \
  (flag & 0x10000 ? '1' : '0'), \
  (flag & 0x20000 ? '1' : '0'), \
  (flag & 0x40000 ? '1' : '0')

//Global Variables ///////////////////////////////////////////////////////////////
int MODE = 0;
int lastMODE = MODE;
double targetMCS[6]  = {0,0,0,0,0,0};
double targetMCSv[6] = {0,0,0,0,0,0};
double readbackMCS[6]  = {0,0,0,0,0,0};
bool transition = false;
float DTI[6] = {0,0,0,0,0,0};
float v[6] = {0,0,0,0,0,0};
//SmarGon Axis # to MCS2 mapping (SmarGon Axis numbering starts at #1)
int AX [] = { 2,  //SmarGon Axis #:
	      2,  // #1
	      5,  // #2
	      0,  // #3
	      1,  // #4
	      4 };// #5
//SA_CTL_DeviceHandle_t dHandle; //Device handle for MCS2 device
ros::Publisher mode_request_pub; //mode request publisher

char    deviceSN[SA_CTL_STRING_MAX_LENGTH]; //I think this is 128
int32_t AX_state[5];
int64_t AX_position[5];
char    AX_state_str[5][20];

// Subscriber Callback functions///////////////////////////////////////////////
// only do the light stuff here in the callback, 
// setting variables, checking input validity
// do all the device communication and error handling in the main loop. 
void modeCallback(const std_msgs::Int16::ConstPtr& msg)
{
    MODE = msg->data;
    transition = true;
    return;
}
void targetMCSCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    targetMCS[0] = msg->position[0];
    targetMCS[1] = msg->position[1];
    targetMCS[2] = msg->position[2];
    targetMCS[3] = msg->position[3];
    targetMCS[4] = msg->position[4];
    targetMCS[5] = msg->position[5];

    targetMCSv[0] = msg->velocity[0];
    targetMCSv[1] = msg->velocity[1];
    targetMCSv[2] = msg->velocity[2];
    targetMCSv[3] = msg->velocity[3];
    targetMCSv[4] = msg->velocity[4];
    targetMCSv[5] = msg->velocity[5];
    //ROS_INFO("MCS2 Set targetMCS");
    return;
}


// Commonly used functions ////////////////////////////////////////////////////
void publishModeRequest (int requestedMode)  {	
	//Create ROS message /readbackMCS and publish
	std_msgs::Int16 msg;
	msg.data = requestedMode;
	mode_request_pub.publish(msg);
	return;
} 	
//function used to react upon an MCS2 communication error
void signalOnError(/*SA_CTL_Result_t result*/) {
   // if (result != SA_CTL_ERROR_NONE) {
   //     SA_CTL_Close(dHandle);
        // Passing an error code to "SA_CTL_GetResultInfo" returns a human readable string
        // specifying the error.
        ROS_INFO("MCS2 %s (error: 0x%04x).",SA_CTL_GetResultInfo(result), result);
    //}
}


void getReadback () {
	//Read out the state and position of each axis fom MCS2:
	//This is done the slightly 'complicated' way, using a "Command Group"
	//(See SmarAct Programmer's Guide) to keep transfer times low.
	//Reading out all axes states and positions with synchronous calls take:
	//2ms per synchrounous call. 10ms for all 5axes. -> 20ms for both states 
	//and positions. With a Command Group, this takes ~2ms
			
	SA_CTL_TransmitHandle_t tHandle;
	SA_CTL_RequestID_t rID[10];

	SA_CTL_OpenCommandGroup(dHandle,&tHandle,SA_CTL_CMD_GROUP_TRIGGER_MODE_DIRECT);
//	    SA_CTL_RequestReadProperty(dHandle,0,SA_CTL_PKEY_NUMBER_OF_CHANNELS,&rID[0],tHandle);
	    
	    SA_CTL_RequestReadProperty(dHandle, AX[1], SA_CTL_PKEY_POSITION, &rID[0], tHandle);
	    SA_CTL_RequestReadProperty(dHandle, AX[2], SA_CTL_PKEY_POSITION, &rID[1], tHandle);
	    SA_CTL_RequestReadProperty(dHandle, AX[3], SA_CTL_PKEY_POSITION, &rID[2], tHandle);
	    SA_CTL_RequestReadProperty(dHandle, AX[4], SA_CTL_PKEY_POSITION, &rID[3], tHandle);
	    SA_CTL_RequestReadProperty(dHandle, AX[5], SA_CTL_PKEY_POSITION, &rID[4], tHandle);

	    SA_CTL_RequestReadProperty(dHandle, AX[1], SA_CTL_PKEY_CHANNEL_STATE,&rID[5],tHandle);
	    SA_CTL_RequestReadProperty(dHandle, AX[2], SA_CTL_PKEY_CHANNEL_STATE,&rID[6],tHandle);
	    SA_CTL_RequestReadProperty(dHandle, AX[3], SA_CTL_PKEY_CHANNEL_STATE,&rID[7],tHandle);
	    SA_CTL_RequestReadProperty(dHandle, AX[4], SA_CTL_PKEY_CHANNEL_STATE,&rID[8],tHandle);
	    SA_CTL_RequestReadProperty(dHandle, AX[5], SA_CTL_PKEY_CHANNEL_STATE,&rID[9],tHandle);
	SA_CTL_CloseCommandGroup(dHandle,tHandle);

        int32_t noOfchannels;
//	SA_CTL_ReadProperty_i32(dHandle, rID[0], &noOfchannels, 0);
        SA_CTL_Result_t result = SA_CTL_ReadProperty_i64(dHandle,rID[0], &AX_position[0], 0);
        if (result != SA_CTL_ERROR_NONE) {
            ROS_INFO("MCS2 Connection Error During Readout of MCS2!");
	    publishModeRequest(99);
        } else {

//	SA_CTL_ReadProperty_i64(dHandle,rID[0], &AX_position[0], 0);
	SA_CTL_ReadProperty_i64(dHandle,rID[1], &AX_position[1], 0);
	SA_CTL_ReadProperty_i64(dHandle,rID[2], &AX_position[2], 0);
	SA_CTL_ReadProperty_i64(dHandle,rID[3], &AX_position[3], 0);
	SA_CTL_ReadProperty_i64(dHandle,rID[4], &AX_position[4], 0);

	SA_CTL_ReadProperty_i32(dHandle,rID[5], &AX_state[0], 0);
	SA_CTL_ReadProperty_i32(dHandle,rID[6], &AX_state[1], 0);
	SA_CTL_ReadProperty_i32(dHandle,rID[7], &AX_state[2], 0);
	SA_CTL_ReadProperty_i32(dHandle,rID[8], &AX_state[3], 0);
	SA_CTL_ReadProperty_i32(dHandle,rID[9], &AX_state[4], 0);

	//convert the i32 AX_state[i] to a string composed of '0' and '1'
	for (int i=0;i<5;i++) {
	    readbackMCS[i] = AX_position[i]*1e-9;
	    AX_state_str[i][0] = 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING);
	    AX_state_str[i][1] = 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_CLOSED_LOOP_ACTIVE);
	    AX_state_str[i][2] = 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_CALIBRATING);
	    AX_state_str[i][3] = 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_REFERENCING);
	    AX_state_str[i][4] = 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_MOVE_DELAYED);
	    AX_state_str[i][5] = 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_SENSOR_PRESENT);
	    AX_state_str[i][6] = 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_IS_CALIBRATED);
	    AX_state_str[i][7] = 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_IS_REFERENCED);
	    AX_state_str[i][8] = 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_END_STOP_REACHED);
	    AX_state_str[i][9] = 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_RANGE_LIMIT_REACHED);
	    AX_state_str[i][10]= 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_FOLLOWING_LIMIT_REACHED);
	    AX_state_str[i][11]= 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED);
	    AX_state_str[i][12]= 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED);
	    AX_state_str[i][13]= 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_POSITIONER_OVERLOAD);
	    AX_state_str[i][14]= 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_OVER_TEMPERATURE);
	    AX_state_str[i][15]= 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_REFERENCE_MARK);
	    AX_state_str[i][16]= 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_IS_PHASED);
	    AX_state_str[i][17]= 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_POSITIONER_FAULT);
	    AX_state_str[i][18]= 0x30+(bool)(AX_state[0] & SA_CTL_CH_STATE_BIT_AMPLIFIER_ENABLED);
	    AX_state_str[i][19]= 0;
	}
        }
}


void publishReadback (ros::Publisher readbackMCS_pub)  {	
	//Create ROS message /readbackMCS and publish
	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = deviceSN;
    msg.name.resize(6);  //in the name tag, we write the state flags as a string
   	msg.name[0] = AX_state_str[0];
   	msg.name[1] = AX_state_str[1];
   	msg.name[2] = AX_state_str[2];
	msg.name[3] = AX_state_str[3];
   	msg.name[4] = AX_state_str[4];
   	msg.name[5] = "";
	msg.position.resize(6); //in the position tag are the positions
	msg.position[0] = readbackMCS[0];
	msg.position[1] = readbackMCS[1];
	msg.position[2] = readbackMCS[2];
	msg.position[3] = readbackMCS[3];
	msg.position[4] = readbackMCS[4];
	msg.position[5] = 0; //omega
	msg.velocity.resize(6); //in the position tag are the positions
	msg.velocity[0] = v[0];
	msg.velocity[1] = v[1];
	msg.velocity[2] = v[2];
	msg.velocity[3] = v[3];
	msg.velocity[4] = v[4];
	msg.velocity[5] = 0; //omega
	msg.effort.resize(6);
	msg.effort[0] = 0;
	msg.effort[1] = 0;
	msg.effort[2] = 0;
	msg.effort[3] = 0;
	msg.effort[4] = 0;
	msg.effort[5] = 0; //omega
	readbackMCS_pub.publish(msg);
} 	
// read positions from MCS2 and publish a JointState message on /readbackMCS topic
void getAndPublishReadback (ros::Publisher readbackMCS_pub)  {
	//Read device serial number from MCS2:
	char deviceSN[SA_CTL_STRING_MAX_LENGTH];
    	size_t ioStringSize = sizeof(deviceSN);
	int result = SA_CTL_GetProperty_s(dHandle, 0, SA_CTL_PKEY_DEVICE_SERIAL_NUMBER, deviceSN, &ioStringSize);
	if (result != SA_CTL_ERROR_NONE) {
	    ROS_INFO("MCS2 failed to read the device serial number.");
	    //FAILED to read Serial Number -> MCS2 not connected!!
	}
	//Read out state and position of each axis fom MCS2:
	int32_t state[5];
	int64_t position[5];
	int i;
	for (i=0;i<5;i++) {
	    signalOnError(SA_CTL_GetProperty_i32(dHandle, AX[i+1], SA_CTL_PKEY_CHANNEL_STATE, &state[i], 0));
	    signalOnError(SA_CTL_GetProperty_i64(dHandle, AX[i+1], SA_CTL_PKEY_POSITION,   &position[i], 0));
	    //signalOnError(SA_CTL_GetProperty_i64(dHandle, AX[i+1], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT,   &position[i], 0));
	    readbackMCS[i] = position[i]*1e-9;
	    AX_state_str[i][0] = 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING);
	    AX_state_str[i][1] = 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_CLOSED_LOOP_ACTIVE);
	    AX_state_str[i][2] = 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_CALIBRATING);
	    AX_state_str[i][3] = 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_REFERENCING);
	    AX_state_str[i][4] = 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_MOVE_DELAYED);
	    AX_state_str[i][5] = 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_SENSOR_PRESENT);
	    AX_state_str[i][6] = 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_IS_CALIBRATED);
	    AX_state_str[i][7] = 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_IS_REFERENCED);
	    AX_state_str[i][8] = 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_END_STOP_REACHED);
	    AX_state_str[i][9] = 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_RANGE_LIMIT_REACHED);
	    AX_state_str[i][10]= 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_FOLLOWING_LIMIT_REACHED);
	    AX_state_str[i][11]= 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED);
	    AX_state_str[i][12]= 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED);
	    AX_state_str[i][13]= 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_POSITIONER_OVERLOAD);
	    AX_state_str[i][14]= 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_OVER_TEMPERATURE);
	    AX_state_str[i][15]= 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_REFERENCE_MARK);
	    AX_state_str[i][16]= 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_IS_PHASED);
	    AX_state_str[i][17]= 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_POSITIONER_FAULT);
	    AX_state_str[i][18]= 0x30+(bool)(state[0] & SA_CTL_CH_STATE_BIT_AMPLIFIER_ENABLED);
	    AX_state_str[i][19]= 0;
	}
	
	//Create ROS message /readbackMCS and publish
	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = deviceSN;
    msg.name.resize(6);  //in the name tag, we write the state flags as a string
   	msg.name[0] = AX_state_str[0];
   	msg.name[1] = AX_state_str[1];
   	msg.name[2] = AX_state_str[2];
	msg.name[3] = AX_state_str[3];
   	msg.name[4] = AX_state_str[4];
   	msg.name[5] = "";
	msg.position.resize(6); //in the position tag are the positions
	msg.position[0] = readbackMCS[0];
	msg.position[1] = readbackMCS[1];
	msg.position[2] = readbackMCS[2];
	msg.position[3] = readbackMCS[3];
	msg.position[4] = readbackMCS[4];
	msg.position[5] = 0; //omega
	msg.velocity.resize(6); //in the position tag are the positions
	msg.velocity[0] = v[0];
	msg.velocity[1] = v[1];
	msg.velocity[2] = v[2];
	msg.velocity[3] = v[3];
	msg.velocity[4] = v[4];
	msg.velocity[5] = 0; //omega
	msg.effort.resize(6);
	msg.effort[0] = 0;
	msg.effort[1] = 0;
	msg.effort[2] = 0;
	msg.effort[3] = 0;
	msg.effort[4] = 0;
	msg.effort[5] = 0; //omega
	readbackMCS_pub.publish(msg);
}

// Main ///////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    
    // Set Up ROS: /////////////////////////////////////////////////////////////
    // You must call one of the versions of ros::init() before using any other
    // part of the ROS system. It uses argc and argv.
    ros::init(argc, argv, "SmarGonMCS2");
    // NodeHandle is the main access point to communications with the ROS 
    // system. The first NodeHandle constructed will fully initialize this node,
    // and the last NodeHandle destructed will close down the node.
    ros::NodeHandle n;
 
    // Advertise Publishers and Subscribers:
    ros::Subscriber mode_sub = n.subscribe("mode",1000, modeCallback);
    ros::Subscriber targetMCS_sub = n.subscribe("targetMCS",1000, targetMCSCallback);
    mode_request_pub = n.advertise<std_msgs::Int16>("mode_request", 1000);
    ros::Publisher readbackMCS_pub = n.advertise<sensor_msgs::JointState>("readbackMCS", 1000);
   
    // Set Up SmarAct MCS2 Controller: /////////////////////////////////////////
    // Read the version of the SmarAct SDK (SA_CTL)
    // Note: this is the only function that does not require the library to be 
    // initialized.
    const char *version = SA_CTL_GetFullVersionString();
    ROS_INFO("SmarActCTL library version: %s.", version);

    // MCS2 devices are identified with locator strings.
    // It is also possible to list all available devices using the 
    // "SA_CTL_FindDevices" function.
    //char locator[] = { "usb:ix:0" }; //USB
    char locator[] = { "usb:sn:MCS2-00002922" }; //USB
    //char locator[] = { "192.168.1.200:55550" }; //Ethernet

    // Additional configuration parameters (unused for now)
    char config[] = {""};

    // All SA_CTL functions return a result code of the type "SA_CTL_Result_t".
    // The return value indicates if the call was successful (SA_CTL_ERROR_NONE)
    // or if an error occurred.
    SA_CTL_Result_t result;

    //ROS loop frequency [Hz]: 
    ros::Rate loop_rate(50);

    int count = 0;
    bool connectedMCS = false;
    bool referenced [6] = {false, false, false, false, false, false};
    bool reference_dir [6] = {0, 0, 0, 0, 0, 0};
    int  referencing_step = 0;
    int  referencing_counter = 0;

    while (ros::ok())
    {
        //detect MODE Transition (used for first time)
	if (MODE==lastMODE) transition=false;
	else transition = true;
	lastMODE = MODE;

        //MAIN SWITCH for the MODE //////////////////////////////////////////////////////////////////////////////////////////	
	switch(MODE) {
            case 0: {//Uninitialized (Connect to MCS)////////////////////////////////////////////////////////////////////////
		if (transition && connectedMCS) { // if already connected, first close connection in order to reconnect
		    //Stop all motion first
	    	    result = SA_CTL_Stop(dHandle, AX[1],0);
	    	    result = SA_CTL_Stop(dHandle, AX[2],0);
	    	    result = SA_CTL_Stop(dHandle, AX[3],0);
	    	    result = SA_CTL_Stop(dHandle, AX[4],0);
	    	    result = SA_CTL_Stop(dHandle, AX[5],0);
		    //Close all existing connections
		    result = SA_CTL_Close(dHandle);
		    ROS_INFO("MCS2 SA_CTL_Close: %d (0 is good)", result);	
		    connectedMCS = false;
		    ROS_INFO("Now in MODE 0: UNINITIALIZED");
		}
		if (!connectedMCS) {
		    // Try to open the MCS2 device
		    result = SA_CTL_Open(&dHandle, locator, config);
		    //if connection failed:
		    if (result != SA_CTL_ERROR_NONE) {
		        ROS_INFO("MCS2 failed to open \"%s\". Error Code: 0x%04x.",locator, result);
		        connectedMCS = false;
			publishModeRequest(99); //switch to error mode 
		    //if connection worked:
		    } else { 
		        ROS_INFO("MCS2 connection opened on locator: \"%s\".", locator);
		        ROS_INFO("MCS2 SA_CTL_Open: result: %d (0 is good)", result);
		    	connectedMCS = true;
		        // Read device serial number, using "SA_CTL_GetProperty_s".
		        char deviceSN[SA_CTL_STRING_MAX_LENGTH];
    		        size_t ioStringSize = sizeof(deviceSN);
		        result = SA_CTL_GetProperty_s(dHandle, 0, SA_CTL_PKEY_DEVICE_SERIAL_NUMBER, deviceSN, &ioStringSize);
		        if (result != SA_CTL_ERROR_NONE) {
		            ROS_INFO("MCS2 failed to read the device serial number.");
			    publishModeRequest(99); //switch to error mode 
    		        } else {
		            ROS_INFO("MCS2 device serial number: \"%s\".", deviceSN);
		        }
		        // Read the number of channels of the system using "SA_CTL_GetProperty_i32".
		        // Note that the "idx" parameter is unused for this property and thus must be set to zero.
		        int32_t noOfchannels;
		        result = SA_CTL_GetProperty_i32(dHandle, 0, SA_CTL_PKEY_NUMBER_OF_CHANNELS, &noOfchannels, 0);
		        if (result != SA_CTL_ERROR_NONE) {
		            ROS_INFO("MCS2 failed to get number of channels.");
		        } else {
		            ROS_INFO("MCS2 number of channels: %d.", noOfchannels);
		            ROS_INFO("MCS2 Ready for Initialization.");
		        }	
		    }
		} else { //If connection established, but still in MODE 0.
		    //Check if connection is alive by continuosly reading out device serial number
		    //if no serial number comes back, assume connection error.
		     //char deviceSN[SA_CTL_STRING_MAX_LENGTH];
    		     size_t ioStringSize = sizeof(deviceSN);
		     result = SA_CTL_GetProperty_s(dHandle, 0, SA_CTL_PKEY_DEVICE_SERIAL_NUMBER, deviceSN, &ioStringSize);
		     if (result != SA_CTL_ERROR_NONE) {  // if connection lost:
		        ROS_INFO("MCS2 CONNECTION LOSS!");
		        connectedMCS = false;
			publishModeRequest(99);
    		     } else { // if connection is still ok:
	                //Read out current positions and publish on /readbackMCS
		        
			getReadback();
		        publishReadback(readbackMCS_pub);
		        //getAndPublishReadback(readbackMCS_pub);
		     }	
		}
		
	    } break;
	    case 1: {//Referencing////////////////////////////////////////////////////////////////////////////////////////////////
		if (transition) { //when fresly transitioned into MODE, start referencing.
		    //Set all axes to unreferenced:
		    referenced[0]=false; referenced[1]=false; referenced[2]=false; 
		    referenced[3]=false; referenced[4]=false; referenced[5]=false; 

		    ROS_INFO("Now in MODE 1: INITIALIZING...");

		    referencing_step = 1; //next step
		}
		switch (referencing_step) {
		    case 1: //Launch Referencing Procedure: Each axis can be set individually
		        referencing_counter = 0;
		        if (referenced[5] == false) {
	 	   	    //result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT, 0);
	 	   	    result = SA_CTL_SetProperty_i32(dHandle, AX[5], SA_CTL_PKEY_REFERENCING_OPTIONS, 0b00100010);
		   	    result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_MOVE_VELOCITY, 0);
		   	    result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		  		    result = SA_CTL_Reference(dHandle, AX[5], 0);
		        }
		        if (referenced[1] == false) {
	 	   	    //result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT, 0);
	 	   	    result = SA_CTL_SetProperty_i32(dHandle, AX[1], SA_CTL_PKEY_REFERENCING_OPTIONS, 0b00100010);
		   	    result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_MOVE_VELOCITY, 0);
		   	    result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		  		    result = SA_CTL_Reference(dHandle, AX[1], 0);
		        }
		        if (referenced[2] == false) {
	 	   	    //result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT, 0);
	 	   	    result = SA_CTL_SetProperty_i32(dHandle, AX[2], SA_CTL_PKEY_REFERENCING_OPTIONS, 0b00100010);
		   	    result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_MOVE_VELOCITY, 0);
		   	    result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		  		    result = SA_CTL_Reference(dHandle, AX[2], 0);
		        }
		        if (referenced[4] == false) {
	 	   	    //result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT, 0);
	 	   	    result = SA_CTL_SetProperty_i32(dHandle, AX[4], SA_CTL_PKEY_REFERENCING_OPTIONS, 0b00000000);
		   	    result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_MOVE_VELOCITY, 0);
		   	    result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		  		    result = SA_CTL_Reference(dHandle, AX[4], 0);
		        }
		        ros::Duration(0.8).sleep();
		        if (referenced[3] == false) {
	 	   	    //result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT, 0);
	 	   	    result = SA_CTL_SetProperty_i32(dHandle, AX[3], SA_CTL_PKEY_REFERENCING_OPTIONS, 0b00000010);
		   	    result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_MOVE_VELOCITY, 0);
		   	    result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		  		    result = SA_CTL_Reference(dHandle, AX[3], 0);
		        }
		        ROS_INFO("Starting Referencing Moves...");
	                referencing_step = 2; //ok, referencing started. Move on.
		   	ros::Duration(1).sleep();
		    break;
		    case 2: //Constantly check state flage
		   	int32_t state [6];
		        for (int i=1;i<=5;i++){ //read out states of all axes
		            int32_t referenced_old;
			    referenced_old = referenced[i];
		   	    result = SA_CTL_GetProperty_i32(dHandle, AX[i], SA_CTL_PKEY_CHANNEL_STATE, &state[i],0);
		   	    if (state[i] & SA_CTL_CH_STATE_BIT_IS_REFERENCED)  {
		   		    referenced[i] = true;
		   		    if (referenced_old == false && referenced[i]==true){
		  			    ROS_INFO(STATEFLAGS_PATTERN, STATEFLAGS(state[i]));
					    ROS_INFO("AXIS %d: IS REFERENCED", i);
				    }
		   	    }
		        }
		        if (referenced[1] && referenced[2] && referenced[3] &&
		            referenced[4] && referenced[5]) {
		   	    referenced[0] = true;
		   	    referencing_step = 3;
		        }
		        referencing_counter++;
		        if (referencing_counter > 100) {
			    ROS_INFO("Referencing timed out, trying again...");
			    //Set all axes to unreferenced:
			    referenced[0]=false; referenced[1]=false; referenced[2]=false; 
			    referenced[3]=false; referenced[4]=false; referenced[5]=false; 
		   	    referencing_step = 1;
		        }
		    break;
		    case 3: //Idle
	           	//Read out current positions and publish on /readbackMCS
		        getReadback();
		        publishReadback(readbackMCS_pub);
		        //getAndPublishReadback(readbackMCS_pub);
			publishModeRequest(2);
		    break;
		}
	    } break;
	    case 2: {// READY Follow Target /////////////////////////////////////////////////////////////////////////////////////////
		if (transition) {
		    // Set to absolute move mode
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[1], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[2], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[3], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[4], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[5], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
		    // Set to 'normal' actuator mode
		    result = SA_CTL_SetProperty_i32(dHandle, AX[1], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[2], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[3], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[4], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[5], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    // Set Move velocity
		    result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_MOVE_VELOCITY, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_MOVE_VELOCITY, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_MOVE_VELOCITY, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_MOVE_VELOCITY, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_MOVE_VELOCITY, 0);
        	    // Set move acceleration [in pm/s2].
		    result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
	    	    // Set following error limit to 0 (0 means disable following error, factory default is 1e9 [pm])
		    result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT,  0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT,  0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT,  0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT,  0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT,  0);

		    //intialize integrator
		    for (int i=0; i<5; i++)
		        DTI[i]=0.;
		    ROS_INFO("Now in MODE 2: READY (Follow Target)");
		}
		
	        //Read out current positions ->readbackMCS[] and publish on /readbackMCS
		getReadback();
		
		//Calculate move velocity based on targetMCS[] and readbackMCS[]
		float kP = 20.; // Proportional Gain
		float kDTI = 0.; // Integrator Gain
		for (int i=0; i<5;i++) {  //Update integrator for each axis
 		    DTI[i] += targetMCS[i]-readbackMCS[i];
		}
		for (int i=0;i<5;i++) { //set velocity for each axis
		    v[i] = kP*(targetMCS[i]-readbackMCS[i]) + kDTI*DTI[i]; //in [mm/s]
		    v[i] = fabs(v[i]); //make positive
		    if (v[i]>100) v[i]=100;  //max velocity (SmarAct Programmer's Guide) : 100x10^9 pm/s = 100mm/s
		}
		
		//Create a 'Command Group' to set velocities and issue move command in one go.
		//See SmarAct Programmer's Guide for details on this procedure.
		//Here, if the the 5 set velocity calls were synchronous, it would take 10ms for all axes.
		//The 5 move calls take 0.8ms all together. Synchronous calls take 10.8ms per cycle.
		//With a 'Command Group' call, this only takes 2ms, all together.
		SA_CTL_TransmitHandle_t tHandle;
		SA_CTL_RequestID_t rID[6];

		SA_CTL_OpenCommandGroup(dHandle,&tHandle,SA_CTL_CMD_GROUP_TRIGGER_MODE_DIRECT);	
		    SA_CTL_RequestWriteProperty_i64(dHandle, AX[1], SA_CTL_PKEY_MOVE_VELOCITY, (int64_t)(v[0]*1e9),&rID[0],tHandle);
		    SA_CTL_RequestWriteProperty_i64(dHandle, AX[2], SA_CTL_PKEY_MOVE_VELOCITY, (int64_t)(v[1]*1e9),&rID[1],tHandle);
		    SA_CTL_RequestWriteProperty_i64(dHandle, AX[3], SA_CTL_PKEY_MOVE_VELOCITY, (int64_t)(v[2]*1e9),&rID[2],tHandle);
		    SA_CTL_RequestWriteProperty_i64(dHandle, AX[4], SA_CTL_PKEY_MOVE_VELOCITY, (int64_t)(v[3]*1e9),&rID[3],tHandle);
		    SA_CTL_RequestWriteProperty_i64(dHandle, AX[5], SA_CTL_PKEY_MOVE_VELOCITY, (int64_t)(v[4]*1e9),&rID[4],tHandle);
		    SA_CTL_Move(dHandle, AX[1], (int64_t)(targetMCS[0]*1e9), tHandle); //[mm] to [nm]: *1e-6  
		    SA_CTL_Move(dHandle, AX[2], (int64_t)(targetMCS[1]*1e9), tHandle);
		    SA_CTL_Move(dHandle, AX[3], (int64_t)(targetMCS[2]*1e9), tHandle);
		    SA_CTL_Move(dHandle, AX[4], (int64_t)(targetMCS[3]*1e9), tHandle);
		    SA_CTL_Move(dHandle, AX[5], (int64_t)(targetMCS[4]*1e9), tHandle);
		SA_CTL_CloseCommandGroup(dHandle,tHandle);

		SA_CTL_WaitForWrite(dHandle,rID[0]);
		SA_CTL_WaitForWrite(dHandle,rID[1]);
		SA_CTL_WaitForWrite(dHandle,rID[2]);
		SA_CTL_WaitForWrite(dHandle,rID[3]);
		SA_CTL_WaitForWrite(dHandle,rID[4]);

		//Now that the stuff that had to get done fast is out, we can publish the Readback values over ROS.
		publishReadback(readbackMCS_pub);
		
	    } break;
	    case 3: { //SIMULATION MODE
	        if (transition) {
			ROS_INFO("Now in MODE 3: SIMULATION");
		}
		for (int i=0;i<6;i++) {
			readbackMCS[i] = targetMCS[i];
		}
		publishReadback(readbackMCS_pub);		
            } 
    	    break;
	    case 4: {}//Command Mode
    		break;
	    case 5: {}//Maintenance (TBD)
    		break;
            case 6: {//MOVEHOME /////////////////////////////////////////////////////////////////////////////////////////////////////
		if (transition) {
		    //Set to absolute move mode
		    result = SA_CTL_SetProperty_i32(dHandle, AX[1], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[2], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[3], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[4], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[5], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
		    //Set to 'normal' actuator mode
		    result = SA_CTL_SetProperty_i32(dHandle, AX[1], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[2], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[3], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[4], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[5], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    // Set move velocity [in pm/s]
		    result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_MOVE_VELOCITY, 5000000000);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_MOVE_VELOCITY, 5000000000);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_MOVE_VELOCITY, 5000000000);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_MOVE_VELOCITY, 5000000000);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_MOVE_VELOCITY, 5000000000);
            	    // Set move acceleration [in pm/s2].
 		    result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
 		    result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
 		    result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
 		    result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
 		    result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		    //Do actual move:
		    int64_t moveValue;
		    moveValue = 0e9;
		    result = SA_CTL_Move(dHandle, AX[1], moveValue, 0);
		    ROS_INFO("SA_CTL_Move AX[1]: %ld, result: %d (0 is good)", moveValue, result);
		    result = SA_CTL_Move(dHandle, AX[2], moveValue, 0);
		    ROS_INFO("SA_CTL_Move AX[2]: %ld, result: %d (0 is good)", moveValue, result);
		    result = SA_CTL_Move(dHandle, AX[3], moveValue, 0);
		    ROS_INFO("SA_CTL_Move AX[3]: %ld, result: %d (0 is good)", moveValue, result);
		    result = SA_CTL_Move(dHandle, AX[4], moveValue, 0);
		    ROS_INFO("SA_CTL_Move AX[4]: %ld, result: %d (0 is good)", moveValue, result);
		    result = SA_CTL_Move(dHandle, AX[5], moveValue, 0);
		    ROS_INFO("SA_CTL_Move AX[5]: %ld, result: %d (0 is good)", moveValue, result);	
		}
	        //Idle
	        //Read out current positions and publish on /readbackMCS
		getAndPublishReadback(readbackMCS_pub);

	    } break;
	    case 7:{} //Calibrate
//		    if (transition) {
//			    status = Smarpod_Calibrate(id);
//			    ROS_INFO("Smarpod_Calibrate: %d", status);
//			    transition = false;
//		    }
		break;
	    case 8:{ //SINETEST ////////////////////////////////////////////////////////////////////////////////////////////
		if (transition) {
		    //Set to absolute move mode
		    result = SA_CTL_SetProperty_i32(dHandle, AX[1], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[2], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[3], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[4], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
    		    result = SA_CTL_SetProperty_i32(dHandle, AX[5], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
		    //Set to 'normal' actuator mode
		    result = SA_CTL_SetProperty_i32(dHandle, AX[1], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[2], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[3], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[4], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    result = SA_CTL_SetProperty_i32(dHandle, AX[5], SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
		    // Set move velocity [in pm/s]
		    result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_MOVE_VELOCITY, 5000000000);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_MOVE_VELOCITY, 5000000000);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_MOVE_VELOCITY, 5000000000);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_MOVE_VELOCITY, 5000000000);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_MOVE_VELOCITY, 5000000000);
            	    // Set move acceleration [in pm/s2].
 		    result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
 		    result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
 		    result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
 		    result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
 		    result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_MOVE_ACCELERATION, 0);
		}
		
		//Read out current positions and publish on /readbackMCS
		//	getAndPublishReadback(readbackMCS_pub);
		  
		{
		    //Do actual move:
		    int64_t position;
		    int64_t velocity;
		    int64_t A = 1e9;
		    double f = 0.3;
		    double secs =ros::Time::now().toSec();
		    position = (int)(0e9+A*sin(2*3.1416*f*secs));
		    velocity = (int)fabs(3.1416*f*2.*A*cos(2*3.1416*f*secs));
		    //velocity = 1e12;
		    result = SA_CTL_SetProperty_i64(dHandle, AX[1], SA_CTL_PKEY_MOVE_VELOCITY, velocity);
		    result = SA_CTL_Move(dHandle, AX[1], position, 0);
		    ROS_INFO("%ld",  position);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[2], SA_CTL_PKEY_MOVE_VELOCITY, velocity);
		    result = SA_CTL_Move(dHandle, AX[2], position, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[3], SA_CTL_PKEY_MOVE_VELOCITY, velocity);
		    result = SA_CTL_Move(dHandle, AX[3], position, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[4], SA_CTL_PKEY_MOVE_VELOCITY, velocity);
		    result = SA_CTL_Move(dHandle, AX[4], position, 0);
		    result = SA_CTL_SetProperty_i64(dHandle, AX[5], SA_CTL_PKEY_MOVE_VELOCITY, velocity);
		    result = SA_CTL_Move(dHandle, AX[5], position, 0);
		    //ROS_INFO("SA_CTL_Move: result: %d (0 is good)", result);
		}



//		    if (!referenced) {  //if not yet referenced, request mode 1 first
//                        //request mode 1
//                       	std_msgs::Int16 msg;
//                       	msg.data = 1;
//                       	mode_request_pub.publish(msg);
//		    }
//		    status = Smarpod_SetSpeed(id,1,speed);
//		    //ROS_INFO("Smarpod_SetSpeed: %d : %.6f", status, speed);
//		    Smarpod_Pose pose;
//		    pose.positionX=targetMCS[0];
//        	    pose.positionY=targetMCS[1];
//        	    pose.positionZ=targetMCS[2];
//        	    pose.rotationX=targetMCS[3];
//        	    pose.rotationY=targetMCS[4];
//        	    pose.rotationZ=targetMCS[5];
//        	    status = Smarpod_Move(id,&pose,SMARPOD_HOLDTIME_INFINITE,0);
//	            ROS_INFO("Smarpod_Move: %d : %.2f", status,targetMCS[0]);
//		    if (status != 0) {
//		    	connectedMCS = false;
//		        ROS_INFO("LOST CONNECTION TO MCS: %d", status);

//                       	std_msgs::Int16 msg;
//                       	msg.data = 0;
//                       	mode_request_pub.publish(msg);
//		    } 
		   } break;
	    case 99: {//ERROR
		if (transition && connectedMCS) { // if already connected, first close connection in order to reconnect
		    //Stop all motion first
	    	    result = SA_CTL_Stop(dHandle, AX[1],0);
	    	    result = SA_CTL_Stop(dHandle, AX[2],0);
	    	    result = SA_CTL_Stop(dHandle, AX[3],0);
	    	    result = SA_CTL_Stop(dHandle, AX[4],0);
	    	    result = SA_CTL_Stop(dHandle, AX[5],0);
		    //Close all existing connections
		    result = SA_CTL_Close(dHandle);
		    ROS_INFO("MCS2 SA_CTL_Close: %d (0 is good)", result);	
		    connectedMCS = false;
		    ROS_INFO("Now in MODE 99: ERROR");
		    ROS_INFO("Trying to reconnect to MCS...");
		}
		if (!connectedMCS) {
		    // Try to open the MCS2 device
		    result = SA_CTL_Open(&dHandle, locator, config);
		    //if connection failed:
		    if (result != SA_CTL_ERROR_NONE) {
		        ROS_INFO("MCS2 failed to open \"%s\". Error Code: 0x%04x.",locator, result);
		        connectedMCS = false;
		    //if connection worked:
		    } else { 
		        ROS_INFO("MCS2 connection opened on locator: \"%s\".", locator);
		        ROS_INFO("MCS2 SA_CTL_Open: result: %d (0 is good)", result);
		    	connectedMCS = true;
		        // Read device serial number, using "SA_CTL_GetProperty_s".
		        char deviceSN[SA_CTL_STRING_MAX_LENGTH];
    		        size_t ioStringSize = sizeof(deviceSN);
		        result = SA_CTL_GetProperty_s(dHandle, 0, SA_CTL_PKEY_DEVICE_SERIAL_NUMBER, deviceSN, &ioStringSize);
		        if (result != SA_CTL_ERROR_NONE) {
		            ROS_INFO("MCS2 failed to read the device serial number.");
    		        } else {
		            ROS_INFO("MCS2 device serial number: \"%s\".", deviceSN);
		            // Read the number of channels of the system using "SA_CTL_GetProperty_i32".
		            // Note that the "idx" parameter is unused for this property and thus must be set to zero.
		            int32_t noOfchannels;
		            result = SA_CTL_GetProperty_i32(dHandle, 0, SA_CTL_PKEY_NUMBER_OF_CHANNELS, &noOfchannels, 0);
		            if (result != SA_CTL_ERROR_NONE) {
		                ROS_INFO("MCS2 failed to get number of channels.");
		            } else {
		                ROS_INFO("MCS2 number of channels: %d.", noOfchannels);
				ROS_INFO("MCS2 Connection Established.");
				//now try to get status of the axes:
				SA_CTL_TransmitHandle_t tHandle;
				SA_CTL_RequestID_t rID[10];
			
				SA_CTL_OpenCommandGroup(dHandle,&tHandle,SA_CTL_CMD_GROUP_TRIGGER_MODE_DIRECT);
				    SA_CTL_RequestReadProperty(dHandle, AX[1], SA_CTL_PKEY_POSITION, &rID[0], tHandle);
				    SA_CTL_RequestReadProperty(dHandle, AX[2], SA_CTL_PKEY_POSITION, &rID[1], tHandle);
				    SA_CTL_RequestReadProperty(dHandle, AX[3], SA_CTL_PKEY_POSITION, &rID[2], tHandle);
				    SA_CTL_RequestReadProperty(dHandle, AX[4], SA_CTL_PKEY_POSITION, &rID[3], tHandle);
				    SA_CTL_RequestReadProperty(dHandle, AX[5], SA_CTL_PKEY_POSITION, &rID[4], tHandle);
			
				    SA_CTL_RequestReadProperty(dHandle, AX[1], SA_CTL_PKEY_CHANNEL_STATE,&rID[5],tHandle);
				    SA_CTL_RequestReadProperty(dHandle, AX[2], SA_CTL_PKEY_CHANNEL_STATE,&rID[6],tHandle);
				    SA_CTL_RequestReadProperty(dHandle, AX[3], SA_CTL_PKEY_CHANNEL_STATE,&rID[7],tHandle);
				    SA_CTL_RequestReadProperty(dHandle, AX[4], SA_CTL_PKEY_CHANNEL_STATE,&rID[8],tHandle);
				    SA_CTL_RequestReadProperty(dHandle, AX[5], SA_CTL_PKEY_CHANNEL_STATE,&rID[9],tHandle);
				SA_CTL_CloseCommandGroup(dHandle,tHandle);
			
			        int32_t noOfchannels;
			        SA_CTL_Result_t result = SA_CTL_ReadProperty_i64(dHandle,rID[0], &AX_position[0], 0);
			        if (result != SA_CTL_ERROR_NONE) {
			            ROS_INFO("Connection MCS2 - SmarGon lost.");
		       		    transition = true;
			        } else {				
		       		    ROS_INFO("Errors Resolved. Requesting MODE 0...");
		       		    publishModeRequest(0);
				}
		            }
                        }
		    }
		}		
    	    } break;
	    default:
    		break;
	}

        ros::spinOnce();
        loop_rate.sleep();

        ++count;
    }
    return 0;
}

