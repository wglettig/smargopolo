// SmarGon MCS2 Geometrical Model for ROS
// Wayne Glettig, 18.9.2020
// 
// This node listens for jointstates in UCS and SCS and outputs the others
// Listeners:
// sensor_msgs/JointState /targetUCS    : target position in UCS
// sensor_msgs/JointState /readbackMCS  : read back position of the MCS2 in MCS
// Publishers:
// sensor_msgs/JointState /targetMCS    : target position in MCS
// sensor_msgs/JointState /readbackUCS  : read back position of the MCS2 converted to UCS
////////////////////////////////////////////////////////////////////////////////

//ROS Includes:
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

//Other Includes:
#include <cmath>
#include <sstream>

//Global Variables
ros::Publisher joint_states_pub;
double OMEGA=0;

double calcTheta(double q3, double q4){
	//relevent lengths of Smargon:
	double l31 = 11.5; 
	double l32 = 68.5 - (80.0)/2.;
	double l33 = l31;
	double l34 = l32;
	double l41 = 76.5;
	double l42 = 25.2;
	double l51 = 10.0;
	double l52 = 2.5;
	double l61 = 64.422;
	double l71 = 5;
	double l72 = 17.67;
	double l73 = 5.2;
	
	
	// Calculate theta (based on q3 & q4)
        // Use the newton method:
        // Starting value theta_0:
        double theta = 0;
        // Set loop conditions
        bool loopcond = true;
        int loopcounter = 0;
        int maxloops = 30;
        while (loopcond) {
                double f, f_diff;
                // Newton Formula, using f, and f_diff, 
                f = sqrt(
                        (l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
                       *(l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
                    +   (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
                       *(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
                    ) - l61;
                f_diff = (2*(cos(theta)*(l71 + l73) + l72*sin(theta))
                           *(l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
                           + 2*(sin(theta)*(l71 + l73) - l72*cos(theta))
                           *(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta)))
                        /(2*sqrt(
                            (l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
                           *(l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))
                         +  (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
                           *(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))
                           ));

                theta = theta - (f/f_diff);
                // Increase loop counter, and stop looping if maxloop is reached
                loopcounter ++;
                if (loopcounter> maxloops){
                        loopcond = false;
        		ROS_INFO("No theta found: q3=%f, q4=%f",q3,q4); 
			return 0.;
                        //ERROR no solution found!!!!
                }
                // Calculate residual error, and if sufficiently small stop looping
                if (fabs(f/f_diff)<1e-9) {
                        loopcond = false;
                }
        }
	return theta;
}


// Subscriber Callback functions///////////////////////////////////////////////
void readbackMCSCallback(const sensor_msgs::JointState::ConstPtr& in_msg)
{
	//The conversion from /targetMCS to /joint_states is:
	///targetMCS	/joint_states
	// q1		axisomega = -q6*1e-3
	// q2		axis1 = q1*1e-3
	// q3		axis2 = q2*1e-3
	// q4		axis3 = q3*1e-3
	// q5		axistheta = theta
	// q6		axisphi = q5

	//extract MCS values from /targetMCS
	double q1 = in_msg->position[0];
	double q2 = in_msg->position[1];
	double q3 = in_msg->position[2];
	double q4 = in_msg->position[3];
	double q5 = in_msg->position[4];
	double q6 = in_msg->position[5];
	//prepare message for /joint_states
	int64_t position;
	sensor_msgs::JointState out_msg;
	out_msg.header.stamp = ros::Time::now();
        out_msg.name.resize(6);
   	out_msg.name[0] = "axisomega";
   	out_msg.name[1] = "axis1";
   	out_msg.name[2] = "axis2";
	out_msg.name[3] = "axis3";
   	out_msg.name[4] = "axistheta";
   	out_msg.name[5] = "axisphi";

	out_msg.position.resize(6);
	out_msg.position[0] = -OMEGA*(M_PI/180);
	out_msg.position[1] = (q1+2.0)*1e-3;
	out_msg.position[2] = q2*1e-3;
	out_msg.position[3] = q3*1e-3;
	out_msg.position[4] = calcTheta(q3,q4);
	out_msg.position[5] = q5*(M_PI/180);
	joint_states_pub.publish(out_msg);
   
	//ROS_INFO("joint_states published");
	return;
}


void readbackOMEGACallback(const std_msgs::Float32::ConstPtr& in_msg)
{
	//extract OMEGA values from /targetOMEGA
	OMEGA = in_msg->data;
	return;
}
// Main ///////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    
    // Set Up ROS: /////////////////////////////////////////////////////////////
    // You must call one of the versions of ros::init() before using any other
    // part of the ROS system. It uses argc and argv.
    ros::init(argc, argv, "readbackMCS_TO_joint_states");
    // NodeHandle is the main access point to communications with the ROS 
    // system. The first NodeHandle constructed will fully initialize this node,
    // and the last NodeHandle destructed will close down the node.
    ros::NodeHandle n;
 
    // Advertise Publishers and Subscribers:
    ros::Subscriber readbackMCS_sub = n.subscribe("readbackMCS",1000, readbackMCSCallback);
    ros::Subscriber readbackOMEGA_sub = n.subscribe("readbackOMEGA",1000, readbackOMEGACallback);
    joint_states_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
  
    //ROS loop frequency [Hz]: 
    ros::Rate loop_rate(50);

    int count = 0;

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        ++count;
    }
    return 0;
}

