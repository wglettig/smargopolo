// Moderator
// Wayne Glettig, 14.4.2020
// 
// This node handles the MODE switching. It listens for mode requests on /mode_request
// and publishes the reigning mode on /mode
//
// It has the following ROS topics interfaces:
// Listeners:
// std_msgs/Int16 /mode_request : the mode requests coming from other nodes
// Publishers:
// std_msgs/Int16 /mode  : the mode to publish to all

//ROS Libraries:
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/JointState.h"



int MODE = 0;
bool latch = false; //this variable is used to latch the publish trigger.
bool is_referenced = false;

void modeRequestCallback(const std_msgs::Int16::ConstPtr& msg)
{
	//only do the light stuff here in the callback, 
	//setting variables, checking input validity
	//do all the device communication and error handling in the main loop. 
	switch (msg->data) {
		case 0: //UNINITIALIZED
			MODE = 0;
			latch = true;
		break;
		case 1: //INITIALIZING
			MODE = 1;
			latch = true;
		break;
		case 2: //READY
			MODE = 2;
			latch = true;
		break;
		case 3: //SIMULATION
			MODE = 3;
			latch = true;
		break;

		case 99: //ERROR
			MODE = 99;
			latch = true;
		break;
		default:
			MODE = msg->data;
		break;
	}
    return;
}

void readbackMCSCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	//Get MCS axis status
	bool is_ref[5];
	is_ref[0] = msg->name[0][7]-0x30;
	is_ref[1] = msg->name[1][7]-0x30;
	is_ref[2] = msg->name[2][7]-0x30;
	is_ref[3] = msg->name[3][7]-0x30;
	is_ref[4] = msg->name[4][7]-0x30;
	if (is_ref[0]&&is_ref[1]&&is_ref[2]&&is_ref[3]&&is_ref[4]) {
		is_referenced = true;
	} else {
		is_referenced = false;
	}

}

int main(int argc, char **argv)
{
    // Initialize ROS:
    ros::init(argc, argv, "Moderator");
    ros::NodeHandle n;
 
    // Advertise Publishers and Subscribers:
    ros::Subscriber mode_request_sub = n.subscribe("mode_request",1000, modeRequestCallback);
    ros::Subscriber readbackMCS_sub = n.subscribe("readbackMCS",1000, readbackMCSCallback);
    ros::Publisher mode_pub = n.advertise<std_msgs::Int16>("mode", 1000);
   
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
	if (latch) {
		std_msgs::Int16 msg;
		msg.data = MODE;
		mode_pub.publish(msg);
		latch = false;
	}


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

