// Traj
// Wayne Glettig, 12.10.2020
// 
// This node handles the Trajectory Generation. It listens for mode requests on /targetSCS_request
// and publishes a smooth trajectory on /targetSCS.
// Upon enabling, it takes the current
//
// It has the following ROS topics interfaces:
// Listeners:
// sensor_msgs/JointState /targetSCS_request : the target position requests coming from other nodes
// Publishers:
// sensor_msgs/JointState /targetSCS : the smoothed series of target positions to feed the Geom Model
//
// It also uses the Parameters of the smargopolo: velocity and acceleration of each axis
// The parameters are loaded, when the node is enabled.

//ROS Libraries:
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/JointState.h"

//sgn function (used in traj)
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


//Global Variables:
int MODE = 0;
bool transition = false;

double xT[9]; //Target value (input)
//State variables:
bool traj_reached[9];
bool enabled[9];
bool firsttime = true; //this variable is used to latch the publish trigger.
//Parameters:
double vmax[9];
double amax[9];
double dt;
//Integrators: (Laufvariablen)
double x[9]; 
double DTI[9];
double DTF[9];
//Output Variables:
double O_x[9];
double O_v[9];
double O_a[9];

int traj_initialize();
int traj_reset();
int traj_runstep();



void modeCallback(const std_msgs::Int16::ConstPtr& msg) {
    MODE = msg->data;
    transition = true;
    return;

} 

void targetSCSRequestCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	//Set MCS target position
	xT[0] = msg->position[0]; enabled[0] = true;
	xT[1] = msg->position[1]; enabled[1] = true;
	xT[2] = msg->position[2]; enabled[2] = true;
	xT[3] = msg->position[3]; enabled[3] = true;
	xT[4] = msg->position[4]; enabled[4] = true;
	xT[5] = msg->position[5]; enabled[5] = true;
	xT[6] = msg->position[6]; enabled[6] = true;
	xT[7] = msg->position[7]; enabled[7] = true;
	xT[8] = msg->position[8]; enabled[8] = true;
	if (firsttime) { traj_reset(); firsttime=false; }
}


int traj_initialize(ros::NodeHandle n) 
{
	dt = 1./50.;
        
        bool ok = true;

        if (!n.getParam("axis/SHX/vmax", vmax[0])){ }
        if (!n.getParam("axis/SHX/amax", amax[0])){ }
        if (!n.getParam("axis/SHY/vmax", vmax[1])){ }
        if (!n.getParam("axis/SHY/amax", amax[1])){ }
        if (!n.getParam("axis/SHZ/vmax", vmax[2])){ }
        if (!n.getParam("axis/SHZ/amax", amax[2])){ }
        if (!n.getParam("axis/OMEGA/vmax", vmax[3])){ }
        if (!n.getParam("axis/OMEGA/amax", amax[3])){ }
        if (!n.getParam("axis/CHI/vmax", vmax[4])){ }
        if (!n.getParam("axis/CHI/amax", amax[4])){ }
        if (!n.getParam("axis/PHI/vmax", vmax[5])){ }
        if (!n.getParam("axis/PHI/amax", amax[5])){ }
        if (!n.getParam("axis/OX/vmax", vmax[6])){ }
        if (!n.getParam("axis/OX/amax", amax[6])){ }
        if (!n.getParam("axis/OY/vmax", vmax[7])){ }
        if (!n.getParam("axis/OY/amax", amax[7])){ }
        if (!n.getParam("axis/OZ/vmax", vmax[8])){ }
        if (!n.getParam("axis/OZ/amax", amax[8])){ }

        ////Old: hard coded values
	//vmax[0]=10; //[mm/s] SHX
	//amax[0]=10; //[mm/s^2]
	//vmax[1]=10; //[mm/s] SHY
	//amax[1]=10; //[mm/s^2]
	//vmax[2]=10; //[mm/s] SHZ
	//amax[2]=10; //[mm/s^2]
	//vmax[3]=10; //[deg/s] OMEGA
	//amax[3]=10; //[deg/s^2]
	//vmax[4]=15; //[deg/s] CHI
	//amax[4]=30; //[deg/s^2]
	//vmax[5]=40; //[deg/s] PHI
	//amax[5]=40; //[deg/s^2]
	//vmax[6]=10; //[mm/s] OX
	//amax[6]=10; //[mm/s^2]
	//vmax[7]=10; //[mm/s] OY
	//amax[7]=10; //[mm/s^2]
	//vmax[8]=10; //[mm/s] OZ
	//amax[8]=10; //[mm/s^2]

	return 0;
}


int traj_reset()
{
	for (int i=0; i<9; i++) {
		//Reset all integrators
		x[i] = xT[i];
		DTI[i] = 0.0;
		DTF[i] = 0.0;
		//Reset all output positions
		O_x[i] = 0.0;
		O_v[i] = 0.0;
		O_a[i] = 0.0;
	}
//	printf("Traj Reset\n");	
	return 0;
}


int traj_runstep()
{
    for (int i=0; i<9; i++){
        if (enabled[i]) { //only update axes that are active
		//Lokale Laufvariablen
		double tempX1, tempDTF, tempU;
		double Y, sigma, m, S;

		tempX1 = DTI[i];
		O_x[i] = tempX1;
    
		tempDTF = (1/dt) * xT[i];
		tempDTF += (-1/dt) * DTF[i];
    
		x[i] += (dt/2) * tempX1;

		//Implementation for the sliding control law
		Y = tempX1 - tempDTF;
		sigma = 1.0 / (dt * amax[i]) * ((x[i] - xT[i]) / dt + Y / 2.0);
		
		m = 0.5 * (1.0 + sqrt(1.0 + 8.0 * fabs(sigma)));
		S = 0.0;

		if (sigma > 0.0)
		    S = 1.0;
		else if (sigma < 0.0)
		    S = -1.0;
		sigma = (Y / (dt * amax[i]) + sigma / m) + (m - 1.0) / 2.0 * S;
		     
		Y = 0.0;
		if (sigma > 0.0)
		    Y = 1.0;
		else if (sigma < 0.0)
		    Y = -1.0;

		Y = (tempX1 * Y + vmax[i]) - dt * amax[i];

		S = 0.0;
		if (Y > 0.0)
		    S = 1.0;
		else if (Y < 0.0)
		    S = -1.0;
	
		if (sigma >= 1.0)
		    Y = 1.0;
		else if (sigma <= -1.0)
		    Y = -1.0;
		else
		    Y = sigma;

		tempU = -0.5 * amax[i] * Y * (1.0 + S);

		//Update Outputs
		O_a[i] = tempU;
		O_v[i] = DTI[i];
		O_x[i] = x[i];
		    
		//Update States 
		DTI[i] += dt * tempU;
		DTF[i] = xT[i] + 0 * DTF[i]; //here an integrator term could be added instead of 0
		x[i] += (dt/2.0) * tempX1;
		
		//Check if Target Position is reached
		if (fabs(xT[i]-O_x[i])<0.001 && fabs(O_v[i])<0.001){
			x[i]   = xT[i]; O_x[i] = xT[i];
			DTI[i] = 0;     O_v[i] = 0;
			DTF[i] = 0;	O_a[i] = 0;
    			enabled[i]=false;
		}
	}
    }
    return 0;
}


int main(int argc, char **argv)
{
    // Initialize ROS:
    ros::init(argc, argv, "smargonTraj");
    ros::NodeHandle n;
 
    // Advertise Publishers and Subscribers:
    ros::Subscriber targetSCS_request_sub = n.subscribe("targetSCS_request",1000, targetSCSRequestCallback);
    ros::Subscriber mode_sub = n.subscribe("mode",1000, modeCallback);
    ros::Publisher targetSCS_pub = n.advertise<sensor_msgs::JointState>("targetSCS", 1000);
   
    ros::Rate loop_rate(50);
    
    traj_initialize(n);

    while (ros::ok())
    {

        if (transition){
           traj_initialize(n);
           transition = false;
        } 
	if (enabled) {
		traj_runstep();
    		//Create SCStarget message
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
    		msg.position[0] = O_x[0];
    		msg.position[1] = O_x[1];
    		msg.position[2] = O_x[2];
    		msg.position[3] = O_x[3];
    		msg.position[4] = O_x[4];
    		msg.position[5] = O_x[5];
    		msg.position[6] = O_x[6];
    		msg.position[7] = O_x[7];
    		msg.position[8] = O_x[8];

		msg.velocity.resize(9);
    		msg.velocity[0] = O_v[0];
    		msg.velocity[1] = O_v[1];
    		msg.velocity[2] = O_v[2];
    		msg.velocity[3] = O_v[3];
    		msg.velocity[4] = O_v[4];
    		msg.velocity[5] = O_v[5];
    		msg.velocity[6] = O_v[6];
    		msg.velocity[7] = O_v[7];
    		msg.velocity[8] = O_v[8];

		targetSCS_pub.publish(msg);
	}

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

