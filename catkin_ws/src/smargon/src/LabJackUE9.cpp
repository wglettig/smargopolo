// LabJack UE9 OMEGA Readout
// Dominik Buntschu, 24.08.2021
// 
// This node handles communication with LabJackUE9 USB DAQ device.
// It provides the following ROS topics interfaces:
// Listeners:
// Publishers:i
// sensor_msgs/JointState   /LJUE9_JointState
// std_msgs/Float32         /LJ_AIN0
// std_msgs/Float32         /LJ_AIN1
// std_msgs/Float32         /LJ_AIN2
// std_msgs/Float32         /readbackOMEGA
// ---std_msgs/Int32           /LJ_QUAD--- DISABLED FOR THE MOMENT
////////////////////////////////////////////////////////////////////////////////

//ROS Includes:
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"

//LabJack UE9 Includes:
extern "C" {
#include "ue9.h"
} 


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


//Globale Variablen
ros::Publisher LJUE9_JointState_pub; //LJUE9_JointState publisher
sensor_msgs::JointState msgJS;

int errorCheck(uint8 *buffer);
float LJ_AIN0 = 0;
float LJ_AIN1 = 0;
float LJ_AIN2 = 0;
float DEG, RADIUS;
double  cycle_us;
int16_t enabled = 0;
int16_t MODE =0;

// ROS Subscription Functions ///////////////////////////////////////////////////
void modeCallback(const std_msgs::Int16::ConstPtr& msg)
{
    MODE = msg->data;
    //transition = true;
    return;
}

// ROS Publishing Functions ///////////////////////////////////////////////////
void prepare_LJUE9_JointState_msg() {
    msgJS.name.resize(4);
    msgJS.name[0] = "OMEGA_DEG";
    msgJS.name[1] = "RADIUS";
    msgJS.name[2] = "AIN0";
    msgJS.name[3] = "AIN1";
    msgJS.position.resize(4);
    return;
}
void publish_LJUE9_JointState()  {	
    msgJS.header.stamp = ros::Time::now();
    msgJS.position[0] = DEG;
    msgJS.position[1] = RADIUS;
    msgJS.position[2] = LJ_AIN0;
    msgJS.position[3] = LJ_AIN1;
    LJUE9_JointState_pub.publish(msgJS);
    return;
} 	

// LabJack Funktionen
int initializeLabJack(HANDLE *hDevice, ue9CalibrationInfo *caliInfo) {

    //Opening first found UE9 over USB
    //ROS_INFO("LJUE9 Opening LabJackUE9...");	
    if( (*hDevice = openUSBConnection(-1)) == NULL ){
	ROS_INFO("LJUE9 ERROR: Failed to open LabJackUE9. Check if USB cable is connected.");
        return -1;	
    }

    ROS_INFO("LJUE9 Opened.");
    ROS_INFO("LJUE9 Getting Calibration Information...");	
    //Getting calibration information from UE9
    if( getCalibrationInfo(*hDevice, caliInfo) < 0 ){
        closeUSBConnection(*hDevice);
	ROS_INFO("LJUE9 ERROR: Failed to get calibration information.");
	return -1;
    }
    ROS_INFO("LJUE9 Ok.");
    
    return 0;
}

int sendAIrequest(HANDLE hDevice, ue9CalibrationInfo *caliInfo) {
    uint8 sendBuff[16], ainResolution;
    uint16 bytesVoltage;
    int sendChars, recChars;
    double voltage;
    ainResolution = 15; //12bit is standard UE9 Pro has up to 18bit

    /* Reading voltage from AIN0 */
    sendBuff[1] = (uint8)(0xA3);  //Command byte
    sendBuff[2] = (uint8)(0x04);  //IOType = 4 (analog in)
    sendBuff[3] = (uint8)(0x00);  //Channel = 0 (AIN0)
    sendBuff[4] = (uint8)(0x08);  //BipGain (Bip = unipolar, Gain = 1) Ori ) 0x00 (0x08 is  bipolar)
    sendBuff[5] = (uint8)(ainResolution);  //Resolution in bit 
    sendBuff[6] = (uint8)(0x00);  //SettlingTime = 0
    sendBuff[7] = (uint8)(0x00);  //Reserved
    sendBuff[0] = normalChecksum8(sendBuff, 8);

    /* Reading voltage from AIN1 */
    sendBuff[1+8] = (uint8)(0xA3);  //Command byte
    sendBuff[2+8] = (uint8)(0x04);  //IOType = 4 (analog in)
    sendBuff[3+8] = (uint8)(0x01);  //Channel = 0 (AIN0)
    sendBuff[4+8] = (uint8)(0x08);  //BipGain (Bip = unipolar, Gain = 1) Ori ) 0x00 (0x08 is  bipolar)
    sendBuff[5+8] = (uint8)(ainResolution);  //Resolution in bit 
    sendBuff[6+8] = (uint8)(0x00);  //SettlingTime = 0
    sendBuff[7+8] = (uint8)(0x00);  //Reserved
    sendBuff[0+8] = normalChecksum8(&sendBuff[8], 8);

    //Send command to UE9
    sendChars = LJUSB_Write(hDevice, sendBuff, 8);
    if( sendChars < 8 )
    {
        if( sendChars == 0 ){
            ROS_INFO("Send Error: write failed\n");
            return -1;
	} else {
            ROS_INFO("Send Error: did not write all of the buffer\n");
            return -1;
	}
    }
    sendChars = LJUSB_Write(hDevice, &sendBuff[8], 8);
    if( sendChars < 8 )
    {
        if( sendChars == 0 ){
            ROS_INFO("Send Error: write failed\n");
            return -1;
	} else {
            ROS_INFO("Send Error: did not write all of the buffer\n");
            return -1;
	}
    }
    return 0;
}

int readAIresponse(HANDLE hDevice, ue9CalibrationInfo *caliInfo) {
    uint8 recBuff[16], ainResolution;
    uint16 bytesVoltage0, bytesVoltage1;
    int recChars;
    double voltage0, voltage1;
    ainResolution = 15; //12bit is standard UE9 Pro has up to 18bit

    //Read response from UE9
    recChars = LJUSB_Read(hDevice, recBuff, 8);
    if( recChars < 8 )
    {
        if( recChars == 0 ) {
            ROS_INFO("Read Error: read failed\n");
            return -1;
	} else {
            ROS_INFO("Read Error: did not read all of the buffer. recChars: %d\n",recChars);
            return -1;
	}
    }
    recChars = LJUSB_Read(hDevice, &recBuff[8], 8);
    if( recChars < 8 )
    {
        if( recChars == 0 ) {
            ROS_INFO("Read Error: read failed\n");
            return -1;
	} else {
            ROS_INFO("Read Error: did not read all of the buffer. recChars: %d\n",recChars);
            return -1;
	}
    }
    //Check for errors
    if( (uint8)(normalChecksum8(recBuff, 8)) != recBuff[0] ) {
        ROS_INFO("Error: read buffer has bad checksum\n");
        return -1;
    }
    if( recBuff[1] != (uint8)(0xA3) ) {
        ROS_INFO("Error: read buffer has wrong command byte\n");
        return -1;
    }
    if( recBuff[2] != (uint8)(0x04) ) {
        ROS_INFO("Error: read buffer has wrong IOType\n");
        return -1;
    }
    if( recBuff[3] != (uint8)(0x00) ) {
        ROS_INFO("Error: read buffer has wrong channel\n");
        return -1;
    }
    bytesVoltage0 = recBuff[5] + recBuff[6]*256;
    bytesVoltage1 = recBuff[5+8] + recBuff[6+8]*256;

    if( getAinVoltCalibrated(caliInfo, 0x08, ainResolution, bytesVoltage0, &voltage0) < 0 )
        return -1;
    if( getAinVoltCalibrated(caliInfo, 0x08, ainResolution, bytesVoltage1, &voltage1) < 0 )
        return -1;

    //update internal variables
    LJ_AIN0=voltage0;
    LJ_AIN1=voltage1;

    return 0;
}

int errorCheck(uint8 *buffer)
{
    uint16 checksumTotal;

    checksumTotal = extendedChecksum16(buffer, 40);
    if( (uint8)((checksumTotal / 256) & 0xFF) != buffer[5] )
    {
        ROS_INFO("Error : read buffer has bad checksum16(MSB)\n");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xFF) != buffer[4] )
    {
        ROS_INFO("Error : read buffer has bad checksum16(LSB)\n");
        return -1;
    }

    if( extendedChecksum8(buffer) != buffer[0] )
    {
        ROS_INFO("Error : read buffer has bad checksum8\n");
        return -1;
    }

    if( buffer[1] != (uint8)(0xF8) || buffer[2] != (uint8)(0x11) || buffer[3] != (uint8)(0x18) )
    {
        ROS_INFO("Error : read buffer has wrong command bytes \n");
        return -1;
    }

    if( buffer[6] != 0 )
    {
        ROS_INFO("Errorcode (byte 6): %d\n", (unsigned int)buffer[6]);
        return -1;
    }

    return 0;
}


// Main Funktion
int main(int argc, char **argv)
{
    
    // Set Up ROS:

    ros::init(argc, argv, "LabJackUE9");
    ros::NodeHandle n; //Accesspoint to talk with ros
 
    // Advertise Publishers and Subscribers:
    LJUE9_JointState_pub = n.advertise<sensor_msgs::JointState>("LJUE9_JointState", 1000);
    prepare_LJUE9_JointState_msg();
    ros::Subscriber mode_sub = n.subscribe("mode",1000, modeCallback);
  
    // Set Up LabJackUE9
    HANDLE hDevice;
    ue9CalibrationInfo caliInfo;

    
    //ROS loop Frequenz [Hz]: 
    int frequency = 200;
    ros::Rate loop_rate(frequency);
    int counter =0;

    int connected=0;
    int transition=0;
    int response;
    while (ros::ok())
    { 

        if (MODE != 3) { //MODE 3 Deactivates this module
            //If not yet connected or disconnected, try to reestablish USB connection every 0.5s (100cycles)
        	if (!connected && (counter>100)) {
                counter=0;
                //closeUSBConnection(hDevice); //close old connection
		if (transition) {
    		    ROS_INFO("LJUE9 Opening LabJackUE9...");	
		    transition=0;
		}
                response = initializeLabJack(&hDevice, &caliInfo);
                if (response != 0) {
                    connected = 0;
                } else {
                    connected = 1;
                    transition = 1;
                } 
            }
                
            if (connected) {

                response = sendAIrequest(hDevice, &caliInfo); //takes about 200us
                if (response != 0) connected = 0;
                
                response = readAIresponse(hDevice, &caliInfo);//takes about 2ms
                if (response != 0) connected = 0;


            }
            
            if (connected) {
            // Calculate angle DEG from cos/sin signals (A0 = cos, A1 = sin)
                float REF_A0 = LJ_AIN0;	
                float REF_A1 = LJ_AIN1;
                float GAIN_A0 = 5/5.002;
                float OFFSET_A0 = -0.005;    
                float GAIN_A1 = 5/5.005;
                float OFFSET_A1= -0.005;
                DEG = atan2(REF_A1*GAIN_A1+OFFSET_A1,REF_A0*GAIN_A0+OFFSET_A0)*180/M_PI;
                if (DEG < 0){
                       DEG = DEG + 360;
                }
                
                RADIUS = sqrt(((REF_A1*GAIN_A1+OFFSET_A1)*(REF_A1*GAIN_A1+OFFSET_A1))+((REF_A0*GAIN_A0+OFFSET_A0)*(REF_A0*GAIN_A0+OFFSET_A0)));

                //ROS_INFO("Publishing on /LJ_AIN0,/LJ_AIN1,/LJ_AIN2,/readbackOMEGA at %d[Hz]", frequency);	
                publish_LJUE9_JointState(); //takes about 40us

                if (transition) {
                    ROS_INFO("LJUE9 Publishing on /LJUE9_JointState: [OMEGA_DEG,RADIUS,AIN0,AIN1 at %d[Hz]", frequency);	
                	transition = 0;
                }	
            }
	}
        counter++;
        ros::spinOnce();
        loop_rate.sleep();

    }

    closeUSBConnection(hDevice);
}
