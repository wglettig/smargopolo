// SmarGon MCS2 Geometrical Model for ROS
// Wayne Glettig, 18.9.2020
// 
// This node listens for jointstates in UCS and SCS and outputs the others
// Listeners:
// sensor_msgs/JointState /targetSCS_corr : target position in SCS (with active correction)
// sensor_msgs/JointState /readbackMCS  : read back position of the MCS2 in MCS
// Publishers:
// sensor_msgs/JointState /targetMCS    : target position in MCS
// sensor_msgs/JointState /readbackSCS  : read back position of the MCS2 converted to SCS
////////////////////////////////////////////////////////////////////////////////

//ROS Includes:
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "ros/package.h"

//SmarGonGM
#include "smargonGM/smargon.h"

//Other Includes:
#include <cmath>
#include <sstream>

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

//Global Variables
smargon MySmarGon;
int MODE;
bool transition=false;
ros::Publisher targetMCS_pub;
ros::Publisher readbackSCS_pub;
double offsetQ4_tweak=0;

// Commonly used functions ////////////////////////////////////////////////////
// read positions from MCS2 and publish a JointState message on /readback topic
void publishTargetMCS (ros::Publisher publisher)  {
	int64_t position;
	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();
        msg.name.resize(9);
   	msg.name[0] = "q1";
   	msg.name[1] = "q2";
   	msg.name[2] = "q3";
	msg.name[3] = "q4";
   	msg.name[4] = "q5";
   	msg.name[5] = "q6";
	
	msg.position.resize(9);
	msg.position[0] = MySmarGon.q1;
	msg.position[1] = MySmarGon.q2;
	msg.position[2] = MySmarGon.q3;
	msg.position[3] = MySmarGon.q4;
	msg.position[4] = MySmarGon.q5;
	msg.position[5] = MySmarGon.q6;

	msg.velocity.resize(9);
	msg.velocity[0] = MySmarGon.q1v;
	msg.velocity[1] = MySmarGon.q2v;
	msg.velocity[2] = MySmarGon.q3v;
	msg.velocity[3] = MySmarGon.q4v;
	msg.velocity[4] = MySmarGon.q5v;
	msg.velocity[5] = MySmarGon.q6v;
	publisher.publish(msg);
}

// Subscriber Callback functions///////////////////////////////////////////////
// only do the light stuff here in the callback, 
// setting variables, checking input validity
// do all the device communication and error handling in the main loop. 
void modeCallback(const std_msgs::Int16::ConstPtr& msg)
{
	//check if new mode is a valid number
	if (msg->data == 0 || 
	    msg->data == 1 ||
	    msg->data == 2 ||
	    msg->data == 3 ||
	    msg->data == 4 ||
	    msg->data == 5 ||
	    msg->data == 6 ||
	    msg->data == 7 ||
	    msg->data == 99) {
		MODE = msg->data;
		transition = true;
	} //ignore if not a valid number
    return;
}


void targetSCSCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    MySmarGon.SHX   = msg->position[0];
    MySmarGon.SHY   = msg->position[1];
    MySmarGon.SHZ   = msg->position[2];
    //MySmarGon.OMEGA = msg->position[3];
    MySmarGon.CHI   = msg->position[4];
    MySmarGon.PHI   = msg->position[5];
    MySmarGon.OX    = msg->position[6];
    MySmarGon.OY    = msg->position[7];
    MySmarGon.OZ    = msg->position[8];
   
    MySmarGon.SHXv   = msg->velocity[0];
    MySmarGon.SHYv   = msg->velocity[1];
    MySmarGon.SHZv   = msg->velocity[2];
    //MySmarGon.OMEGAv = msg->velocity[3];
    MySmarGon.CHIv   = msg->velocity[4];
    MySmarGon.PHIv   = msg->velocity[5];
    MySmarGon.OXv    = msg->velocity[6];
    MySmarGon.OYv    = msg->velocity[7];
    MySmarGon.OZv    = msg->velocity[8];
    //ROS_INFO("MCS2 Set targetSCS");
    MySmarGon.runIKv();
    publishTargetMCS(targetMCS_pub);
    return;
}

void readbackMCSCallback(const sensor_msgs::JointState::ConstPtr& in_msg)
{
    //Get MCS readback values from message
    MySmarGon.q1 = in_msg->position[0];
    MySmarGon.q2 = in_msg->position[1];
    MySmarGon.q3 = in_msg->position[2];
    MySmarGon.q4 = in_msg->position[3];
    MySmarGon.q5 = in_msg->position[4];
    MySmarGon.q6 = in_msg->position[5];
   
    //ROS_INFO("MCS2 Set targetSCS");
    //run FK
    MySmarGon.runFK();
 
    //Create SCS readback message
    int64_t position;
    sensor_msgs::JointState out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.name.resize(9);
    out_msg.name[0] = "SHX";
    out_msg.name[1] = "SHY";
    out_msg.name[2] = "SHZ";
    out_msg.name[3] = "OMEGA";
    out_msg.name[4] = "CHI";
    out_msg.name[5] = "PHI";
    out_msg.name[6] = "OX";
    out_msg.name[7] = "OY";
    out_msg.name[8] = "OZ";
    
    out_msg.position.resize(9);
    out_msg.position[0] = MySmarGon.SHX;
    out_msg.position[1] = MySmarGon.SHY;
    out_msg.position[2] = MySmarGon.SHZ;
    out_msg.position[3] = MySmarGon.OMEGA;
    out_msg.position[4] = MySmarGon.CHI;
    out_msg.position[5] = MySmarGon.PHI;
    out_msg.position[6] = MySmarGon.OX;
    out_msg.position[6] = MySmarGon.OX;
    out_msg.position[7] = MySmarGon.OY;
    out_msg.position[8] = MySmarGon.OZ;
    readbackSCS_pub.publish(out_msg);
    return;
}

void readbackOMEGACallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //Get OMEGA readback value from message
    MySmarGon.OMEGA = msg->position[0];
    return;
}
int loadParameters(smargon *MySmarGon, ros::NodeHandle n) {
    //Get the parameters from the ROS parameter server and populate MySmarGon object

    bool ok = true;

    //set lengths and distances:
    if (!n.getParam("length/l01", MySmarGon->l01)) {ROS_ERROR("Failed to load param 'length/l01'");ok=false;}
    if (!n.getParam("length/l11", MySmarGon->l11)) {ROS_ERROR("Failed to load param 'length/l11'");ok=false;}
    if (!n.getParam("length/l12", MySmarGon->l12)) {ROS_ERROR("Failed to load param 'length/l12'");ok=false;}
    if (!n.getParam("length/l21", MySmarGon->l21)) {ROS_ERROR("Failed to load param 'length/l21'");ok=false;}
    if (!n.getParam("length/l22", MySmarGon->l22)) {ROS_ERROR("Failed to load param 'length/l22'");ok=false;}
    if (!n.getParam("length/l23", MySmarGon->l23)) {ROS_ERROR("Failed to load param 'length/l23'");ok=false;}
    if (!n.getParam("length/l31", MySmarGon->l31)) {ROS_ERROR("Failed to load param 'length/l31'");ok=false;}
    if (!n.getParam("length/l32", MySmarGon->l32)) {ROS_ERROR("Failed to load param 'length/l32'");ok=false;}
    if (!n.getParam("length/l33", MySmarGon->l33)) {ROS_ERROR("Failed to load param 'length/l33'");ok=false;}
    if (!n.getParam("length/l34", MySmarGon->l34)) {ROS_ERROR("Failed to load param 'length/l34'");ok=false;}
    if (!n.getParam("length/l41", MySmarGon->l41)) {ROS_ERROR("Failed to load param 'length/l41'");ok=false;}
    if (!n.getParam("length/l42", MySmarGon->l42)) {ROS_ERROR("Failed to load param 'length/l42'");ok=false;}
    if (!n.getParam("length/l51", MySmarGon->l51)) {ROS_ERROR("Failed to load param 'length/l51'");ok=false;}
    if (!n.getParam("length/l52", MySmarGon->l52)) {ROS_ERROR("Failed to load param 'length/l52'");ok=false;}
    if (!n.getParam("length/l61", MySmarGon->l61)) {ROS_ERROR("Failed to load param 'length/l61'");ok=false;}
    if (!n.getParam("length/l71", MySmarGon->l71)) {ROS_ERROR("Failed to load param 'length/l71'");ok=false;}
    if (!n.getParam("length/l72", MySmarGon->l72)) {ROS_ERROR("Failed to load param 'length/l72'");ok=false;}
    if (!n.getParam("length/l73", MySmarGon->l73)) {ROS_ERROR("Failed to load param 'length/l73'");ok=false;}
    if (!n.getParam("length/l74", MySmarGon->l74)) {ROS_ERROR("Failed to load param 'length/l74'");ok=false;}
    if (!n.getParam("length/offsetQ1", MySmarGon->offsetQ1)) {ROS_ERROR("Failed to load param 'length/offsetQ1'");ok=false;}
    if (!n.getParam("length/offsetQ2", MySmarGon->offsetQ2)) {ROS_ERROR("Failed to load param 'length/offsetQ2'");ok=false;}
    if (!n.getParam("length/offsetQ3", MySmarGon->offsetQ3)) {ROS_ERROR("Failed to load param 'length/offsetQ3'");ok=false;}
    if (!n.getParam("length/offsetQ4", MySmarGon->offsetQ4)) {ROS_ERROR("Failed to load param 'length/offsetQ4'");ok=false;}
    if (!n.getParam("length/offsetQ5", MySmarGon->offsetQ5)) {ROS_ERROR("Failed to load param 'length/offsetQ5'");ok=false;}
    if (!n.getParam("length/offsetQ6", MySmarGon->offsetQ6)) {ROS_ERROR("Failed to load param 'length/offsetQ6'");ok=false;}
    
    //start values:
    if (!n.getParam("axis/q1/start", MySmarGon->q1_start)) {ROS_ERROR("Failed to load param 'axis/q1/start'");ok=false;}
    if (!n.getParam("axis/q2/start", MySmarGon->q2_start)) {ROS_ERROR("Failed to load param 'axis/q2/start'");ok=false;}
    if (!n.getParam("axis/q3/start", MySmarGon->q3_start)) {ROS_ERROR("Failed to load param 'axis/q3/start'");ok=false;}
    if (!n.getParam("axis/q4/start", MySmarGon->q4_start)) {ROS_ERROR("Failed to load param 'axis/q4/start'");ok=false;}
    if (!n.getParam("axis/q5/start", MySmarGon->q5_start)) {ROS_ERROR("Failed to load param 'axis/q5/start'");ok=false;}
    if (!n.getParam("axis/q6/start", MySmarGon->q6_start)) {ROS_ERROR("Failed to load param 'axis/q6/start'");ok=false;}
 
    if (!n.getParam("axis/SHX/start", MySmarGon->SHX_start)) {ROS_ERROR("Failed to load param 'axis/SHX/start'");ok=false;}
    if (!n.getParam("axis/SHY/start", MySmarGon->SHY_start)) {ROS_ERROR("Failed to load param 'axis/SHY/start'");ok=false;}
    if (!n.getParam("axis/SHZ/start", MySmarGon->SHZ_start)) {ROS_ERROR("Failed to load param 'axis/SHZ/start'");ok=false;}
    if (!n.getParam("axis/CHI/start", MySmarGon->CHI_start)) {ROS_ERROR("Failed to load param 'axis/CHI/start'");ok=false;}
    if (!n.getParam("axis/PHI/start", MySmarGon->PHI_start)) {ROS_ERROR("Failed to load param 'axis/PHI/start'");ok=false;}
    if (!n.getParam("axis/OMEGA/start", MySmarGon->OMEGA_start)) {ROS_ERROR("Failed to load param 'axis/OMEGA/start'");ok=false;}
    if (!n.getParam("axis/OX/start", MySmarGon->OX_start)) {ROS_ERROR("Failed to load param 'axis/OX/start'");ok=false;}
    if (!n.getParam("axis/OY/start", MySmarGon->OY_start)) {ROS_ERROR("Failed to load param 'axis/OY/start'");ok=false;}
    if (!n.getParam("axis/OZ/start", MySmarGon->OZ_start)) {ROS_ERROR("Failed to load param 'axis/OZ/start'");ok=false;}

    if (ok) {
        ROS_INFO("smargonGM: Parameters loaded into MySmarGon Object" );
        return 0;
    } else {
        ROS_ERROR("smargonGM: NOT ALL PARAMETERS were found in smargonparam.yaml. Shutting down smargonGM." );
        return 1;
    }
    
    return 0;
}


void tweak_Q4_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    offsetQ4_tweak = msg->data;
    MySmarGon.offsetQ4 = offsetQ4_tweak;
}


// Main ///////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    
    // Set Up ROS: /////////////////////////////////////////////////////////////
    // You must call one of the versions of ros::init() before using any other
    // part of the ROS system. It uses argc and argv.
    ros::init(argc, argv, "SmarGonGM");
    // NodeHandle is the main access point to communications with the ROS 
    // system. The first NodeHandle constructed will fully initialize this node,
    // and the last NodeHandle destructed will close down the node.
    ros::NodeHandle n;
 
    // Advertise Publishers and Subscribers:
    //ros::Publisher targetMCS_pub = n.advertise<sensor_msgs::JointState>("targetMCS", 1000);
    targetMCS_pub = n.advertise<sensor_msgs::JointState>("targetMCS", 1000);
    readbackSCS_pub = n.advertise<sensor_msgs::JointState>("readbackSCS", 1000);
    ros::Subscriber mode_sub = n.subscribe("mode",1000, modeCallback);
    ros::Subscriber targetSCS_sub = n.subscribe("targetSCS_corr",1000, targetSCSCallback);
    ros::Subscriber readbackMCS_sub = n.subscribe("readbackMCS",1000, readbackMCSCallback);
    ros::Subscriber LJUE9_JointState_sub = n.subscribe("LJUE9_JointState",1000, readbackOMEGACallback);
    ros::Subscriber tweak_Q4_sub = n.subscribe("tweak_Q4",1000, tweak_Q4_Callback);
 

    if (loadParameters(&MySmarGon, n)==1) return 1;  //stop program if not all parameters could be loaded in

    ros::spin();

    //ROS loop frequency [Hz]: 
//    ros::Rate loop_rate(50);
//
//    int count = 0;
//
//    while (ros::ok())
//    {
//        ros::spinOnce();
//        loop_rate.sleep();
//
//        ++count;
//    }
    return 0;
}

