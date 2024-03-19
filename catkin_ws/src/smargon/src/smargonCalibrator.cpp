// SmarGon XYZ Calibrator (Strain Gauge) using MCS2
// Dominik Buntschu, 8.7.2021
// 
// This node handles communication with the SmarAct MCS2 and
// provides the following ROS topics interfaces:
// Listeners:
// std_msgs/Int16         /mode         : Operation mode of the smargopolo control system
// std_msgs/Int16         /enableCAL    : Enable Calibrator Closed Loop, 1: ENABLED, 0: DISABLED
// Publishers:
// sensor_msgs/JointState /readbackCAL  : Calibrator Readback position (read from MCS2)
////////////////////////////////////////////////////////////////////////////////

//ROS Includes:
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/JointState.h"

//SmarAct SDK Includes:
#include <SmarActControl.h>

//Other Includes:
#include <cmath>
#include <chrono>

//Chrono can be used to measure the time of certain events in [us]:
//auto start = std::chrono::high_resolution_clock::now();
// //do stuff here//
//auto stop = std::chrono::high_resolution_clock::now();
//auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
//std::cout << duration.count() << std::endl;
//cycle_us = duration.count();
//ROS_INFO("Cycle_us %lf",cycle_us);

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

//Globale Variablen
int MODE = 0;
int32_t enableCAL = 0;
bool transition = false;
int lastMODE = MODE;
double cycle_us;

SA_CTL_DeviceHandle_t dHandle; //Device handle for MCS2 device
ros::Publisher JointState_pub; //JointState publisher for /readbackCAL
sensor_msgs::JointState msgJS;
ros::Publisher mode_request_pub; //mode request publisher

int64_t ch0=0, ch1=0, ch2=0, x=0, y=0, z=0., adc0=0, adc1=0, adc2=0;   // positions read out
char    deviceSN[SA_CTL_STRING_MAX_LENGTH]; //I think this is 128

// Subscriber Callback functions///////////////////////////////////////////////
// only do the light stuff here in the callback, 
// setting variables, checking input validity
// do all the device communication and error handling in the main loop. 
void modeCallback(const std_msgs::Int16::ConstPtr& msg)
{
    MODE = msg->data;
    return;
}
void enableCALCallback(const std_msgs::Int16::ConstPtr& msg)
{
    int32_t enableCAL_old;
    enableCAL_old = enableCAL;

    enableCAL = (bool)msg->data; //update internal global variable

    if (enableCAL_old != enableCAL) //check if logic state of enableCAL has changed
       transition = true;
    return;
}
void moveCALCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (!enableCAL) {
        double moveCAL[3];
        moveCAL[0] = msg->position[0];
        moveCAL[1] = msg->position[1];
        moveCAL[2] = msg->position[2];
    
        for (int i=0;i<3;i++){
            if (moveCAL[i] != 0.0) { //if a non zero move value is detected move there
                //SEND MOVE COMMAND TO MCS2 (OPEN LOOP STEPS -1000 to 1000)
                if (i==2) SA_CTL_Move(dHandle, i, (int64_t)(-moveCAL[i]*1e3), 0);
                else SA_CTL_Move(dHandle, i, (int64_t)(moveCAL[i]*1e3), 0);
                ROS_INFO("MoveCAL %d, %ld",i,(int64_t)(moveCAL[i]*1e3));
	        }
	    }
    }
}



// Commonly used functions ////////////////////////////////////////////////////
int transform(void) {
    // Coordinate transformation from calibrator channel positions to X, Y, Z coordinates
    double RX, RZ;
    RX = -45 *(M_PI/180);
    RZ = -54.736 *(M_PI/180);
    x = -ch1 * sin(RX) * sin(RZ) + ch0 * cos(RX) * sin(RZ) + ch2 * cos(RZ);
    y =  ch2 * sin(RZ) + ch1 * sin(RX) * cos(RZ) - ch0 * cos(RX) * cos(RZ);
    z = -ch0 * sin(RX) - ch1 * cos(RX);

    RX = (-1.29) *(M_PI/180);    //y = y*cos(RX)-z*sin(RX);
    z = y*sin(RX)+z*cos(RX);

    return 0;
}

void prepare_JointState_msg(){
    msgJS.header.frame_id = deviceSN;
    msgJS.name.resize(9);  //in the name tag, we write the state flags as a string
    msgJS.name[0] = "x";
    msgJS.name[1] = "y";
    msgJS.name[2] = "z";
    msgJS.name[3] = "ch0";
    msgJS.name[4] = "ch1";
    msgJS.name[5] = "ch2";
    msgJS.name[6] = "adc0";
    msgJS.name[7] = "adc1";
    msgJS.name[8] = "adc2";
    msgJS.position.resize(9); //in the position tag are the positions
    return;
}

void publishJointState(){	
    msgJS.header.stamp = ros::Time::now();
    msgJS.position[0] = x *1e-9;
    msgJS.position[1] = y *1e-9;
    msgJS.position[2] = z *1e-9;
    msgJS.position[3] = ch0 *1e-9;
    msgJS.position[4] = ch1 *1e-9;
    msgJS.position[5] = ch2 *1e-9;
    msgJS.position[6] = adc0;
    msgJS.position[7] = adc1;
    msgJS.position[8] = adc2;
    JointState_pub.publish(msgJS);
    return;
} 	

//function used to react upon an MCS2 communication error
void signalOnError(SA_CTL_Result_t result) {
    if (result != SA_CTL_ERROR_NONE) {
        SA_CTL_Close(dHandle);
        // Passing an error code to "SA_CTL_GetResultInfo" returns a human readable string
        // specifying the error.
        ROS_INFO("MCS2 %s (error: 0x%04x).",SA_CTL_GetResultInfo(result), result);
    }
}


// Main ///////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    
    // Set Up ROS: /////////////////////////////////////////////////////////////
    ros::init(argc, argv, "smargonCalibrator");
    ros::NodeHandle n;//Accesspoint to talk with ros
 
    // Advertise Publishers and Subscribers:
    ros::Subscriber mode_sub = n.subscribe("mode",1000, modeCallback);
    ros::Subscriber enableCAL_sub = n.subscribe("enableCAL",1000, enableCALCallback);
    ros::Subscriber moveCAL_sub = n.subscribe("moveCAL",1000, moveCALCallback);
    JointState_pub = n.advertise<sensor_msgs::JointState>("readbackCAL", 1000);
    prepare_JointState_msg();
   
    // Set Up SmarAct MCS2 Controller: /////////////////////////////////////////
    // Read the version of the SmarAct SDK (SA_CTL)
    const char *version = SA_CTL_GetFullVersionString();
    ROS_INFO("SmarActCTL library version: %s.", version);

    // MCS2 devices are identified with locator strings.
    // It is also possible to list all available devices using the 
    // "SA_CTL_FindDevices" function.
    char locator[] = { "usb:sn:MCS2-00006300" }; //USB
    //char locator[] = { "192.168.1.200:55550" }; //Ethernet

    // Additional configuration parameters (unused for now)
    char config[] = {""};

    // All SA_CTL functions return a result code of the type "SA_CTL_Result_t".
    // The return value indicates if the call was successful (SA_CTL_ERROR_NONE)
    // or if an error occurred.
    SA_CTL_Result_t result;

    // Try to open the MCS2 device
    result = SA_CTL_Open(&dHandle, locator, config);
    //if connection failed:
    if (result != SA_CTL_ERROR_NONE) {
        ROS_INFO("MCS2 failed to open \"%s\". Error Code: 0x%04x.",locator, result);
        //Close all existing connections
        result = SA_CTL_Close(dHandle);	
	return -1;
    }    
    //if connection worked: 
    ROS_INFO("MCS2 connection opened on locator: \"%s\".", locator);
    ROS_INFO("SA_CTL_Open: result: %d (0 is good)", result);

    size_t ioStringSize = sizeof(deviceSN);
    result = SA_CTL_GetProperty_s(dHandle, 0, SA_CTL_PKEY_DEVICE_SERIAL_NUMBER, deviceSN, &ioStringSize);
    if (result != SA_CTL_ERROR_NONE) {
        ROS_INFO("MCS2 failed to read the device serial number.");
        return -1;
        } else {
        ROS_INFO("MCS2 device serial number: \"%s\".", deviceSN);
    }



    // Check the number of Sensors mounted using "SA_CTL_GetProperty_i32".
    for (int i=0; i<3; i++) {
	    int32_t state;
	    result = SA_CTL_GetProperty_i32(dHandle, i, SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
            if (!(state & SA_CTL_CH_STATE_BIT_SENSOR_PRESENT)) {
		    ROS_INFO("MCS2 Three Channels with Sensors are Needed. Failed on %d. Exiting...", i);
       		    //Close all existing connections
       		    result = SA_CTL_Close(dHandle);	
		    return -1;
	    }
    }
        


    // Configure IO module (+-10V range): /////////////////////////////////////////// 
    //Sets the IO module analog input range to Bipolar +/-10V
    result = SA_CTL_SetProperty_i32(dHandle, 0, SA_CTL_PKEY_IO_MODULE_ANALOG_INPUT_RANGE, SA_CTL_IO_MODULE_ANALOG_INPUT_RANGE_BI_10V);

    for (int channel=0; channel<3; channel++){
       // sensors must be enabled (not in power save mode) (setting this propertz stops tehe channel and disables the control loop)
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_SENSOR_POWER_MODE, SA_CTL_SENSOR_MODE_ENABLED);
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MAX_CL_FREQUENCY, 12000); // Hz

       // define input to be used as feedback
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_AUX_IO_MODULE_INPUT_INDEX, 0);
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_AUX_INPUT_SELECT, SA_CTL_AUX_INPUT_SELECT_IO_MODULE); 
       if (channel == 2)
            result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_AUX_DIRECTION_INVERSION, SA_CTL_INVERTED);
       else
            result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_AUX_DIRECTION_INVERSION, SA_CTL_NON_INVERTED);
  
       //route aux input into control loop
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_CONTROL_LOOP_INPUT,  SA_CTL_CONTROL_LOOP_INPUT_AUX_IN);

       // unlock write access to tune the (custom) positioner type
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_WRITE_PROTECTION, SA_CTL_POS_WRITE_PROTECTION_KEY);
       // the endstop detection must be disabled
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_ESD_DIST_TH, 0);
       //_set control loop gains
       
       // TODO tune to best performance
       //result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_PID_SHIFT, 1);
       //result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_P_GAIN, 1000);
       //result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_D_GAIN, 100);
       //result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_I_GAIN, 0);
       //result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_ANTI_WINDUP,0);
       
       //for accurate measurement with low force
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_PID_SHIFT, 1);
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_P_GAIN, 500);
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_D_GAIN, 10);
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_I_GAIN, 0);
       result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_POS_ANTI_WINDUP,0);

       // note: setting the position refers to the position calculation for the build-in sensors
       result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_POSITION, 0);

       // note: the commanded target is in adc increments now and *not* in 'position'
       // the range is -131072 to 131071 corresponding to +-10.24V input voltage
       // the amplifier box should be offset trimmed to generate ca. ADC = 0 with unstressed force sensors

       // the current adc value may be read with 'Control Loop Input Aux Value'
       int64_t aux;
       result = SA_CTL_GetProperty_i64(dHandle, channel, SA_CTL_PKEY_CL_INPUT_AUX_VALUE, &aux,0);
       ROS_INFO("AUX value: %ld (channel %d)", aux, channel);

    }

    // MAIN ROS LOOP //////////////////////////////////////////////////////////////////////////////////

    //ROS loop frequency [Hz]: 
    ros::Rate loop_rate(200);

    int count = 0;
    while (ros::ok())
    {
        // If enableCAL was changed (transition=true), start or stop control loop:
        if (transition) {
            if (enableCAL) { //start control loop
		//int32_t target =32000; //int(131071 / 4); # ca 25% of the (positive) control range to 'press' against the sensors
	        int32_t target =3000; //for accurate measurements with low force on smargon
                    for (int channel=0; channel<3; channel++){
                    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
                    result = SA_CTL_Move(dHandle, channel, target,0); // enable control loop;
		    }
	    } 
	    if (!enableCAL) { //stop control loop
	        for (int channel=0; channel<3; channel++){
	            result= SA_CTL_Stop(dHandle, channel,0);
                // Set move mode to Open-loop (steps)
                result= SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_STEP);
		    }		    
	    }
            transition = false;
        }	

        //***************************group read**********************************************
        SA_CTL_TransmitHandle_t tHandle;
        SA_CTL_RequestID_t rID[6];

        //readout time of Sensors without ADC Values is about 2ms to 3ms
        //readout time of sensors with ADC Values is about 3.5ms to 4.5ms
       
 
        //auto start = std::chrono::high_resolution_clock::now();//Start timer

        SA_CTL_OpenCommandGroup(dHandle,&tHandle,SA_CTL_CMD_GROUP_TRIGGER_MODE_DIRECT);
            SA_CTL_RequestReadProperty(dHandle, 0, SA_CTL_PKEY_CL_INPUT_SENSOR_VALUE, &rID[0], tHandle);
            SA_CTL_RequestReadProperty(dHandle, 1, SA_CTL_PKEY_CL_INPUT_SENSOR_VALUE, &rID[1], tHandle);
            SA_CTL_RequestReadProperty(dHandle, 2, SA_CTL_PKEY_CL_INPUT_SENSOR_VALUE, &rID[2], tHandle);
           
            //SA_CTL_RequestReadProperty(dHandle, 0, SA_CTL_PKEY_CL_INPUT_AUX_VALUE, &rID[3], tHandle);
            //SA_CTL_RequestReadProperty(dHandle, 1, SA_CTL_PKEY_CL_INPUT_AUX_VALUE, &rID[4], tHandle);
            //SA_CTL_RequestReadProperty(dHandle, 2, SA_CTL_PKEY_CL_INPUT_AUX_VALUE, &rID[5], tHandle);
        SA_CTL_CloseCommandGroup(dHandle,tHandle);


        SA_CTL_ReadProperty_i64(dHandle,rID[0], &ch0, 0);
        SA_CTL_ReadProperty_i64(dHandle,rID[1], &ch1, 0);
        SA_CTL_ReadProperty_i64(dHandle,rID[2], &ch2, 0);

        transform();

        //SA_CTL_ReadProperty_i64(dHandle,rID[3], &adc0, 0);
        //SA_CTL_ReadProperty_i64(dHandle,rID[4], &adc1, 0);
        //SA_CTL_ReadProperty_i64(dHandle,rID[5], &adc2, 0);

        //auto stop = std::chrono::high_resolution_clock::now();//Stop timer
        //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
        //std::cout << duration.count() << std::endl;
        //cycle_us = duration.count();
        //ROS_INFO("Cycle_us %lf",cycle_us);
  
        publishJointState();//takes about 100us

        // ****************************************************************************************
        ros::spinOnce();
        loop_rate.sleep();

        ++count;
    }

    // DO ON EXITING: //////////////////////////////////////////////////////////////////////////////////
    //
    // Stop all motion:
    for (int channel=0; channel<3; channel++){
        result= SA_CTL_Stop(dHandle, channel,0);
    }
    //Close all existing connections
    result = SA_CTL_Close(dHandle);

    return 0;
}
