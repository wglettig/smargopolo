// SmarGon MCS2 BCS to SCS
// Wayne Glettig, 24.4.2021
// 
// This node keeps track of current /readbackSCS and transforms it to /readbackBCS.
//
// Upon a /target_BCS_request, it converts (BX,BY,BZ,CHI,PHI,OMEGA) -> SHY,SHY,SHZ 
// and publishes this to /targetSCS_request
//
// Listeners:
// sensor_msgs/JointState /readbackMCS         : read back position in MCS
// sensor_msgs/JointState /targetBCS_request   : target position request in BCS
// sensor_msgs/JointState /LJUE9_JointState    : read back position of OMEGA
// not used://std_msgs/Float32 /readbackOMEGA             : read back position of OMEGA
//
// Publishers:
// sensor_msgs/JointState /readbackBCS         : read back position in BCS
// sensor_msgs/JointState /targetSCS_request   : target position request in SCS
////////////////////////////////////////////////////////////////////////////////

//ROS Includes:
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

//Other Includes:
#include <cmath>


//Global Variables
ros::Publisher targetSCS_request_pub;
ros::Publisher readbackBCS_pub;
double SHX, SHY, SHZ;
double SHX0=0, SHY0=0, SHZ0=18.;
double OMEGA, CHI, PHI;
double OX, OY, OZ;

double BX, BY, BZ;

// Calculation Functions ////////////////////////////////////////////////////
//[BX,BY,BZ,OMEGA,CHI,PHI] -> [SHX,SHY,SHZ]:
int calculateBCS2SCS () {
    double phi, chi, omega;

    //Angle conversion DEG to RAD (smargopolo works in DEG, trigonometric calculations below use RAD):
    phi = PHI/180*M_PI;
    chi = CHI/180*M_PI;
    omega = OMEGA/180*M_PI;

    //calculate the [SHX,SHY,SHZ] from [BX,BY,BZ,OMEGA,CHI,PHI] coordinates:
    SHX= SHX0 + BX*cos(phi)*sin(chi) - BZ*cos(omega)*sin(phi) - BY*sin(omega)*sin(phi) - BY*cos(chi)*cos(omega)*cos(phi) + BZ*cos(chi)*cos(phi)*sin(omega);
    SHY= SHY0 - BZ*cos(omega)*cos(phi) - BY*cos(phi)*sin(omega) - BX*sin(chi)*sin(phi) + BY*cos(chi)*cos(omega)*sin(phi) - BZ*cos(chi)*sin(omega)*sin(phi);
    SHZ= SHZ0 - BX*cos(chi) - BY*cos(omega)*sin(chi) + BZ*sin(chi)*sin(omega);

    return 0;
} 

//[SHX,SHY,SHZ,OMEGA,CHI,PHI] -> [BX,BY,BZ]:
int calculateSCS2BCS () {
    double phi, chi, omega;

    //Angle conversion DEG to RAD (smargopolo works in DEG, trigonometric calculations below use RAD):
    phi = PHI/180*M_PI;
    chi = CHI/180*M_PI;
    omega = OMEGA/180*M_PI;

    //calculate the [SHX,SHY,SHZ] from [BX,BY,BZ,OMEGA,CHI,PHI] coordinates:
    BX= SHZ0*cos(chi) - SHZ*cos(chi) + SHX*cos(phi)*sin(chi) - SHX0*cos(phi)*sin(chi) - SHY*sin(chi)*sin(phi) + SHY0*sin(chi)*sin(phi);
    BY= SHX0*(sin(omega)*sin(phi) + cos(chi)*cos(omega)*cos(phi)) - SHX*(sin(omega)*sin(phi) + cos(chi)*cos(omega)*cos(phi)) - SHY*(cos(phi)*sin(omega) - cos(chi)*cos(omega)*sin(phi)) + SHY0*(cos(phi)*sin(omega) - cos(chi)*cos(omega)*sin(phi)) - SHZ*cos(omega)*sin(chi) + SHZ0*cos(omega)*sin(chi);
    BZ= SHX0*(cos(omega)*sin(phi) - cos(chi)*cos(phi)*sin(omega)) - SHX*(cos(omega)*sin(phi) - cos(chi)*cos(phi)*sin(omega)) - SHY*(cos(omega)*cos(phi) + cos(chi)*sin(omega)*sin(phi)) + SHY0*(cos(omega)*cos(phi) + cos(chi)*sin(omega)*sin(phi)) + SHZ*sin(chi)*sin(omega) - SHZ0*sin(chi)*sin(omega);

    return 0;
} 

// Publisher Functions///////////////////////////////////////////////
//
// Publish a /readbackBCS JointState message
void publishReadbackBCS ()  {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name.resize(6);
        msg.name[0] = "BX";
        msg.name[1] = "BY";
        msg.name[2] = "BZ";
        msg.name[3] = "OMEGA";
        msg.name[4] = "CHI";
        msg.name[5] = "PHI";

        msg.position.resize(6);
        msg.position[0] = BX;
        msg.position[1] = BY;
        msg.position[2] = BZ;
        msg.position[3] = OMEGA;
        msg.position[4] = CHI;
        msg.position[5] = PHI;

        readbackBCS_pub.publish(msg);
}
// Publish a /targetSCS_request JointState message
void publishTargetSCS_request ()  {
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

        targetSCS_request_pub.publish(msg);
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
void readbackSCSCallback(const sensor_msgs::JointState::ConstPtr& msg)
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
    
    calculateSCS2BCS();

    publishReadbackBCS();
    //ROS_INFO("Publishing /readbackBCS");
}
void targetBCS_requestCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    BX   = msg->position[0];
    BY   = msg->position[1];
    BZ   = msg->position[2];
    //OMEGA = msg->position[3];
    CHI   = msg->position[4];
    PHI   = msg->position[5];

    calculateBCS2SCS();

    publishTargetSCS_request();
    //ROS_INFO("Publishing TargetSCS_request");
}



// Main ///////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{

    // Set Up ROS: /////////////////////////////////////////////////////////////
    // You must call one of the versions of ros::init() before using any other
    // part of the ROS system. It uses argc and argv.
    ros::init(argc, argv, "BCS2SCS");
    // NodeHandle is the main access point to communications with the ROS 
    // system. The first NodeHandle constructed will fully initialize this node,
    // and the last NodeHandle destructed will close down the node.
    ros::NodeHandle n;

    // Advertise Publishers and Subscribers:
    //ros::Subscriber readbackSCS_sub = n.subscribe("readbackSCS",1000, readbackSCSCallback);
    ros::Subscriber readbackSCS_sub = n.subscribe("targetSCS",1000, readbackSCSCallback);
    ros::Subscriber targetiBCS_request_sub = n.subscribe("targetBCS_request",1000, targetBCS_requestCallback);
    ros::Subscriber readbackLJUE9_sub = n.subscribe("LJUE9_JointState",1000, readbackLJUE9Callback);
    readbackBCS_pub = n.advertise<sensor_msgs::JointState>("readbackBCS", 1000);
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

