// SmarGon MCS2 Limit to Bounds
// Wayne Glettig, 04.11.2021
// 
// This module listens on the /targetSCS_request and checks if the values are within 
// the bounds specified in the parameter file /axis/.../limpos & limneg
// if a value is outside the bounds, the limit value is republished.
// and a message is posted on rosout
//
// Listeners:
// sensor_msgs/JointState /targetSCS_request  : target position in SCS
//
// Publishers:
// sensor_msgs/JointState /targetSCS_corr     : target position in SCS
////////////////////////////////////////////////////////////////////////////////

//ROS Includes:
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
//Other Includes:
#include <cmath>

//Global Variables
sensor_msgs::JointState input_msg;
sensor_msgs::JointState  output_msg;
ros::Publisher targetSCS_request_pub;
int MODE = 0;
bool transition = false;

double SHX_limpos=0;
double SHX_limneg=0;
double SHY_limpos=0;
double SHY_limneg=0;
double SHZ_limpos=0;
double SHZ_limneg=0;
double CHI_limpos=0;
double CHI_limneg=0;
double PHI_limpos=0;
double PHI_limneg=0;
double OMEGA_limpos=0;
double OMEGA_limneg=0;
double OX_limpos=0;
double OX_limneg=0;
double OY_limpos=0;
double OY_limneg=0;
double OZ_limpos=0;
double OZ_limneg=0;


// Commonly used functions ////////////////////////////////////////////////////
// Load Parameters from Server

int loadParameters(ros::NodeHandle n){
//Get the parameters from the ROS parameter server and save to global varialbes in this module
// (makes reading faster)

bool ok = true;

//get lengths and distances:
if (!n.getParam("axis/SHX/limpos", SHX_limpos)) {ROS_ERROR("Failed to load param 'axis/SHX/limpos'");ok=false;}
if (!n.getParam("axis/SHX/limneg", SHX_limneg)) {ROS_ERROR("Failed to load param 'axis/SHX/limneg'");ok=false;}
if (!n.getParam("axis/SHY/limpos", SHY_limpos)) {ROS_ERROR("Failed to load param 'axis/SHY/limpos'");ok=false;}
if (!n.getParam("axis/SHY/limneg", SHY_limneg)) {ROS_ERROR("Failed to load param 'axis/SHY/limneg'");ok=false;}
if (!n.getParam("axis/SHZ/limpos", SHZ_limpos)) {ROS_ERROR("Failed to load param 'axis/SHZ/limpos'");ok=false;}
if (!n.getParam("axis/SHZ/limneg", SHZ_limneg)) {ROS_ERROR("Failed to load param 'axis/SHZ/limneg'");ok=false;}


if (!n.getParam("axis/CHI/limpos", CHI_limpos)) {ROS_ERROR("Failed to load param 'axis/CHI/limpos'");ok=false;}
if (!n.getParam("axis/CHI/limneg", CHI_limneg)) {ROS_ERROR("Failed to load param 'axis/CHI/limneg'");ok=false;}
if (!n.getParam("axis/PHI/limpos", PHI_limpos)) {ROS_ERROR("Failed to load param 'axis/PHI/limpos'");ok=false;}
if (!n.getParam("axis/PHI/limneg", PHI_limneg)) {ROS_ERROR("Failed to load param 'axis/PHI/limneg'");ok=false;}
if (!n.getParam("axis/OMEGA/limpos", OMEGA_limpos)) {ROS_ERROR("Failed to load param 'axis/OMEGA/limpos'");ok=false;}
if (!n.getParam("axis/OMEGA/limneg", OMEGA_limneg)) {ROS_ERROR("Failed to load param 'axis/OMEGA/limneg'");ok=false;}

if (!n.getParam("axis/OX/limpos", OX_limpos)) {ROS_ERROR("Failed to load param 'axis/OX/limpos'");ok=false;}
if (!n.getParam("axis/OX/limneg", OX_limneg)) {ROS_ERROR("Failed to load param 'axis/OX/limneg'");ok=false;}
if (!n.getParam("axis/OY/limpos", OY_limpos)) {ROS_ERROR("Failed to load param 'axis/OY/limpos'");ok=false;}
if (!n.getParam("axis/OY/limneg", OY_limneg)) {ROS_ERROR("Failed to load param 'axis/OY/limneg'");ok=false;}
if (!n.getParam("axis/OZ/limpos", OZ_limpos)) {ROS_ERROR("Failed to load param 'axis/OZ/limpos'");ok=false;}
if (!n.getParam("axis/OZ/limneg", OZ_limneg)) {ROS_ERROR("Failed to load param 'axis/OZ/limneg'");ok=false;}

if (ok) {
ROS_INFO("Bounds: Parameters successfully loaded." );
return 0;
} else {
ROS_ERROR("Bounds: NOT ALL PARAMETERS were found in smargonparam.yaml. Shutting down bounds." );
return 1;
}


}

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
 
void check_limpos(sensor_msgs::JointState input_msg, sensor_msgs::JointState * output_msg, int i, const char* name, double limitVal, bool* republish) {
    if (input_msg.position[i] > limitVal) {
        output_msg->position[i] = limitVal; 
        ROS_INFO("%s out of bounds: Request: %.3f, POS Limit %.3f, Setting to %.3f.", name, input_msg.position[i], limitVal, limitVal);
        *republish=true;
    }
} 
void check_limneg(sensor_msgs::JointState input_msg, sensor_msgs::JointState * output_msg, int i, const char* name, double limitVal, bool* republish) {
    if (input_msg.position[i] < limitVal) {
        output_msg->position[i] = limitVal; 
        ROS_INFO("%s out of bounds: Request: %.3f, NEG Limit %.3f, Setting to %.3f.", name, input_msg.position[i], limitVal, limitVal);
        *republish=true;
    }
} 
void targetSCS_requestCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // When a /targetSCS message arrives, calculate correction and output 
    // a /targetSCS_corr message, depending on the corr_type.
    input_msg = *msg;
    output_msg = input_msg;    
    
    bool republish = false;
    check_limpos(input_msg, &output_msg, 0, "SHX", SHX_limpos, &republish);
    check_limneg(input_msg, &output_msg, 0, "SHX", SHX_limneg, &republish);
    check_limpos(input_msg, &output_msg, 1, "SHY", SHY_limpos, &republish);
    check_limneg(input_msg, &output_msg, 1, "SHY", SHY_limneg, &republish);
    check_limpos(input_msg, &output_msg, 2, "SHZ", SHZ_limpos, &republish);
    check_limneg(input_msg, &output_msg, 2, "SHZ", SHZ_limneg, &republish);
    check_limpos(input_msg, &output_msg, 3, "OMEGA", OMEGA_limpos, &republish);
    check_limneg(input_msg, &output_msg, 3, "OMEGA", OMEGA_limneg, &republish);
    check_limpos(input_msg, &output_msg, 4, "CHI", CHI_limpos, &republish);
    check_limneg(input_msg, &output_msg, 4, "CHI", CHI_limneg, &republish);
    check_limpos(input_msg, &output_msg, 5, "PHI", PHI_limpos, &republish);
    check_limneg(input_msg, &output_msg, 5, "PHI", PHI_limneg, &republish);
    check_limpos(input_msg, &output_msg, 6, "OX", OX_limpos, &republish);
    check_limneg(input_msg, &output_msg, 6, "OX", OX_limneg, &republish);
    check_limpos(input_msg, &output_msg, 7, "OY", OY_limpos, &republish);
    check_limneg(input_msg, &output_msg, 7, "OY", OY_limneg, &republish);
    check_limpos(input_msg, &output_msg, 8, "OZ", OZ_limpos, &republish);
    check_limneg(input_msg, &output_msg, 8, "OZ", OZ_limneg, &republish);
    
    
    if (republish) {
        targetSCS_request_pub.publish(output_msg); //publish output message
    }
    
    return;
}


// Main ///////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    
    // Set Up ROS: /////////////////////////////////////////////////////////////
    // You must call one of the versions of ros::init() before using any other
    // part of the ROS system. It uses argc and argv.
    ros::init(argc, argv, "bounds");
    // NodeHandle is the main access point to communications with the ROS 
    // system. The first NodeHandle constructed will fully initialize this node,
    // and the last NodeHandle destructed will close down the node.
    ros::NodeHandle n;

    // Advertise Publishers and Subscribers:
    ros::Subscriber targetSCS_request_sub = n.subscribe("targetSCS_request",1000, targetSCS_requestCallback);
    ros::Subscriber mode_sub = n.subscribe("mode",1000, modeCallback);
    targetSCS_request_pub = n.advertise<sensor_msgs::JointState>("targetSCS_request", 1000);

    if (loadParameters(n)==1) return 1;  //stop program if not all parameters could be loaded in

    // ROS Spin.
    ros::spin();

    return 0;
}

