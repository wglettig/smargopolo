// SmarGon MCS2 BCS to MCS
// Wayne Glettig, 24.4.2021
// 
// This node keeps track of current /readbackMCS
// upon a /nudgeBCS message, it takes /readbackMCS and shifts it by the nudge vector,
// then posts a /targetSCS_request
// Listeners:
// // std_msgs/Float32 /readbackOMEGA       : read back position of OMEGA
// sensor_msgs/JointState /LJUE9_JointState : read back position of the OMEGA
// sensor_msgs/JointState /readbackMCS      : read back position of the MCS2 in MCS
// sensor_msgs/JointState /nudgeBCS         : vector to nudge
// Publishers:
// sensor_msgs/JointState /targetSCS_request    : target position request in SCS
////////////////////////////////////////////////////////////////////////////////

//ROS Includes:
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

//Other Includes:
#include <cmath>


//Global Variables
ros::Publisher targetSCS_request_pub;
double SHX, SHY, SHZ;
double OMEGA, CHI, PHI;
double OX, OY, OZ;

double NX, NY, NZ;
//Orientation tweak parameters:
double offsetOMEGA = 0.0;
double FPRX=0.0, FPRY=0.0, FPRZ=0.0;
//double FPRX=-90.0, FPRY=0.0, FPRZ=13.0; //Vespa orientation

// Commonly used functions ////////////////////////////////////////////////////
int calculateSH () {

	double oldSHX, oldSHY, oldSHZ;
	double nudgeSHX, nudgeSHY, nudgeSHZ;
	double phi, chi, omega, offsetOmega, fprx, fpry, fprz;
	//Get existing SH values:
	oldSHX = SHX; oldSHY = SHY; oldSHZ = SHZ;

	//Angle conversion DEG to RAD (smargopolo works in DEG, trigonometric calculations below use RAD):
	phi = PHI/180*M_PI;
	chi = CHI/180*M_PI;
	omega = OMEGA/180*M_PI;
	offsetOmega = offsetOMEGA/180*M_PI;
	fprx = FPRX/180*M_PI;
	fpry = FPRY/180*M_PI;
	fprz = FPRZ/180*M_PI;

	//calculate the nudge vector [NX,NY,NZ] in SH coordinates:
	nudgeSHX = NX*cos(omega + offsetOmega)*sin(fpry)*sin(phi) - NZ*cos(omega + offsetOmega)*cos(fprx)*cos(fpry)*sin(phi) - NX*sin(omega + offsetOmega)*cos(chi)*cos(phi)*sin(fpry) - NY*cos(omega + offsetOmega)*cos(fpry)*sin(fprx)*sin(phi) - NY*sin(omega + offsetOmega)*cos(fprx)*cos(fprz)*sin(phi) - NX*sin(omega + offsetOmega)*cos(fpry)*sin(fprz)*sin(phi) + NZ*sin(omega + offsetOmega)*cos(fprz)*sin(fprx)*sin(phi) + NX*cos(fpry)*cos(fprz)*cos(phi)*sin(chi) - NY*cos(fprx)*cos(phi)*sin(chi)*sin(fprz) + NZ*cos(phi)*sin(chi)*sin(fprx)*sin(fprz) - NZ*sin(omega + offsetOmega)*cos(fprx)*sin(fpry)*sin(fprz)*sin(phi) + NZ*cos(fprx)*cos(fprz)*cos(phi)*sin(chi)*sin(fpry) - NY*sin(omega + offsetOmega)*sin(fprx)*sin(fpry)*sin(fprz)*sin(phi) + NY*cos(fprz)*cos(phi)*sin(chi)*sin(fprx)*sin(fpry) - NY*cos(omega + offsetOmega)*cos(chi)*cos(fprx)*cos(fprz)*cos(phi) - NX*cos(omega + offsetOmega)*cos(chi)*cos(fpry)*cos(phi)*sin(fprz) + NZ*sin(omega + offsetOmega)*cos(chi)*cos(fprx)*cos(fpry)*cos(phi) + NZ*cos(omega + offsetOmega)*cos(chi)*cos(fprz)*cos(phi)*sin(fprx) + NY*sin(omega + offsetOmega)*cos(chi)*cos(fpry)*cos(phi)*sin(fprx) - NZ*cos(omega + offsetOmega)*cos(chi)*cos(fprx)*cos(phi)*sin(fpry)*sin(fprz) - NY*cos(omega + offsetOmega)*cos(chi)*cos(phi)*sin(fprx)*sin(fpry)*sin(fprz);
	nudgeSHY = NX*cos(omega + offsetOmega)*cos(phi)*sin(fpry) - NZ*cos(omega + offsetOmega)*cos(fprx)*cos(fpry)*cos(phi) - NY*cos(omega + offsetOmega)*cos(fpry)*cos(phi)*sin(fprx) - NY*sin(omega + offsetOmega)*cos(fprx)*cos(fprz)*cos(phi) - NX*sin(omega + offsetOmega)*cos(fpry)*cos(phi)*sin(fprz) + NZ*sin(omega + offsetOmega)*cos(fprz)*cos(phi)*sin(fprx) + NX*sin(omega + offsetOmega)*cos(chi)*sin(fpry)*sin(phi) - NX*cos(fpry)*cos(fprz)*sin(chi)*sin(phi) + NY*cos(fprx)*sin(chi)*sin(fprz)*sin(phi) - NZ*sin(chi)*sin(fprx)*sin(fprz)*sin(phi) - NY*sin(omega + offsetOmega)*cos(chi)*cos(fpry)*sin(fprx)*sin(phi) - NZ*sin(omega + offsetOmega)*cos(fprx)*cos(phi)*sin(fpry)*sin(fprz) - NY*sin(omega + offsetOmega)*cos(phi)*sin(fprx)*sin(fpry)*sin(fprz) - NZ*cos(fprx)*cos(fprz)*sin(chi)*sin(fpry)*sin(phi) - NY*cos(fprz)*sin(chi)*sin(fprx)*sin(fpry)*sin(phi) + NY*cos(omega + offsetOmega)*cos(chi)*cos(fprx)*cos(fprz)*sin(phi) + NX*cos(omega + offsetOmega)*cos(chi)*cos(fpry)*sin(fprz)*sin(phi) - NZ*sin(omega + offsetOmega)*cos(chi)*cos(fprx)*cos(fpry)*sin(phi) - NZ*cos(omega + offsetOmega)*cos(chi)*cos(fprz)*sin(fprx)*sin(phi) + NZ*cos(omega + offsetOmega)*cos(chi)*cos(fprx)*sin(fpry)*sin(fprz)*sin(phi) + NY*cos(omega + offsetOmega)*cos(chi)*sin(fprx)*sin(fpry)*sin(fprz)*sin(phi);
	nudgeSHZ = NY*cos(chi)*cos(fprx)*sin(fprz) - NX*sin(omega + offsetOmega)*sin(chi)*sin(fpry) - NX*cos(chi)*cos(fpry)*cos(fprz) - NZ*cos(chi)*sin(fprx)*sin(fprz) - NY*cos(omega + offsetOmega)*cos(fprx)*cos(fprz)*sin(chi) - NX*cos(omega + offsetOmega)*cos(fpry)*sin(chi)*sin(fprz) + NZ*sin(omega + offsetOmega)*cos(fprx)*cos(fpry)*sin(chi) + NZ*cos(omega + offsetOmega)*cos(fprz)*sin(chi)*sin(fprx) + NY*sin(omega + offsetOmega)*cos(fpry)*sin(chi)*sin(fprx) - NZ*cos(chi)*cos(fprx)*cos(fprz)*sin(fpry) - NY*cos(chi)*cos(fprz)*sin(fprx)*sin(fpry) - NZ*cos(omega + offsetOmega)*cos(fprx)*sin(chi)*sin(fpry)*sin(fprz) - NY*cos(omega + offsetOmega)*sin(chi)*sin(fprx)*sin(fpry)*sin(fprz);

	//add the nudge vecor to the existing SH position
	SHX = oldSHX + nudgeSHX;
	SHY = oldSHY + nudgeSHY;
	SHZ = oldSHZ + nudgeSHZ;

	return 0;
}

int calculateO () {

	double oldOX, oldOY, oldOZ, omega, offsetOmega, fprx, fpry, fprz;
	double nudgeOX, nudgeOY, nudgeOZ;
	//Get existing SH values:
	oldOX = OX; oldOY = OY; oldOZ = OZ;

	//calculate the nudge vector [NX,NY,NZ] in SH coordinates:
 	omega = OMEGA/180*M_PI;
	offsetOmega = offsetOMEGA/180*M_PI;
	fprx = FPRX/180*M_PI;
	fpry = FPRY/180*M_PI;
	fprz = FPRZ/180*M_PI;

	//This is the simplified formula without fprx,fpry,frpz, offsetOmega
	//nudgeOX = NY*cos(omega) - NZ*sin(omega);
	//nudgeOY = NY*sin(omega) + NZ*cos(omega);
	//nudgeOZ = NX;

	//This is the formula that takes in account fprx,fpry,frpz and offsetOmega
        nudgeOX = NX*(sin(omega + offsetOmega)*sin(fpry) + cos(omega + offsetOmega)*cos(fpry)*sin(fprz)) + NY*(cos(omega + offsetOmega)*cos(fprx)*cos(fprz) - sin(omega + offsetOmega)*cos(fpry)*sin(fprx) + cos(omega + offsetOmega)*sin(fprx)*sin(fpry)*sin(fprz))+ NZ*(cos(omega + offsetOmega)*cos(fprx)*sin(fpry)*sin(fprz) - cos(omega + offsetOmega)*cos(fprz)*sin(fprx) - sin(omega + offsetOmega)*cos(fprx)*cos(fpry));
        nudgeOY = NX*(sin(omega + offsetOmega)*cos(fpry)*sin(fprz) - cos(omega + offsetOmega)*sin(fpry)) + NY*(cos(omega + offsetOmega)*cos(fpry)*sin(fprx) + sin(omega + offsetOmega)*cos(fprx)*cos(fprz) + sin(omega + offsetOmega)*sin(fprx)*sin(fpry)*sin(fprz))+ NZ*(cos(omega + offsetOmega)*cos(fprx)*cos(fpry) - sin(omega + offsetOmega)*cos(fprz)*sin(fprx) + sin(omega + offsetOmega)*cos(fprx)*sin(fpry)*sin(fprz));
        nudgeOZ = NX*cos(fpry)*cos(fprz) + NY*(cos(fprz)*sin(fprx)*sin(fpry) - cos(fprx)*sin(fprz)) + NZ*(sin(fprx)*sin(fprz) + cos(fprx)*cos(fprz)*sin(fpry));

	//add the nudge vecor to the existing SH position
	OX = oldOX + nudgeOX;
	OY = oldOY + nudgeOY;
	OZ = oldOZ + nudgeOZ;

	return 0;
}


// Publish a /targetSCS_request JointState message
void publishTargetSCS_request (ros::Publisher publisher)  {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name.resize(9);
        msg.name[0] = "SHX";
        msg.name[1] = "SHY";
        msg.name[2] = "SHZ";
        msg.name[3] = "OMEGA";
        msg.name[4] = "CHI";
        msg.name[5] = "PHI";
        msg.name[6] = "OX";
        msg.name[7] = "OY";
        msg.name[8] = "OZ";

        msg.position.resize(9);
        msg.position[0] = SHX;
        msg.position[1] = SHY;
        msg.position[2] = SHZ;
        msg.position[3] = OMEGA;
        msg.position[4] = CHI;
        msg.position[5] = PHI;
        msg.position[6] = OX;
        msg.position[7] = OY;
        msg.position[8] = OZ;

        publisher.publish(msg);
}
// Subscriber Callback functions///////////////////////////////////////////////
// only do the light stuff here in the callback, 
// setting variables, checking input validity
// do all the device communication and error handling in the main loop. 
void readbackLJUE9Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    OMEGA = msg->position[0];
}

//void readbackOMEGACallback(const std_msgs::Float32::ConstPtr& msg)
//{
//    OMEGA = msg->data;
//}
void targetSCS_requestCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    SHX   = msg->position[0];
    SHY   = msg->position[1];
    SHZ   = msg->position[2];
    //OMEGA = msg->position[3];
    CHI   = msg->position[4];
    PHI   = msg->position[5];
    OX    = msg->position[6];
    OY    = msg->position[7];
    OZ    = msg->position[8];
}
void nudgeBCSCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    NX   = msg->position[0];
    NY   = msg->position[1];
    NZ   = msg->position[2];

    calculateSH();

    publishTargetSCS_request(targetSCS_request_pub);
    //ROS_INFO("Publishing TargetSCS_request");
}

void nudgeOBCSCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    NX   = msg->position[0];
    NY   = msg->position[1];
    NZ   = msg->position[2];

    calculateO();

    publishTargetSCS_request(targetSCS_request_pub);
    //ROS_INFO("Publishing TargetSCS_request");
}



// Main ///////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{

    // Set Up ROS: /////////////////////////////////////////////////////////////
    // You must call one of the versions of ros::init() before using any other
    // part of the ROS system. It uses argc and argv.
    ros::init(argc, argv, "Nudge");
    // NodeHandle is the main access point to communications with the ROS 
    // system. The first NodeHandle constructed will fully initialize this node,
    // and the last NodeHandle destructed will close down the node.
    ros::NodeHandle n;

    // Advertise Publishers and Subscribers:
    ros::Subscriber readbackLJUE9_sub = n.subscribe("LJUE9_JointState",1000, readbackLJUE9Callback);
    ros::Subscriber targetSCS_request_sub = n.subscribe("targetSCS_request",1000, targetSCS_requestCallback);
    ros::Subscriber nudgeBCS_sub = n.subscribe("nudgeBCS",1000, nudgeBCSCallback);
    ros::Subscriber nudgeOBCS_sub = n.subscribe("nudgeOBCS",1000, nudgeOBCSCallback);
    targetSCS_request_pub = n.advertise<sensor_msgs::JointState>("targetSCS_request", 1000);

    ros::spin();

    //ROS loop frequency [Hz]: 
    /*ros::Rate loop_rate(100);


    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }*/
    return 0;
}

