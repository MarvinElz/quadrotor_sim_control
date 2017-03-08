#include "ros/ros.h"

// for Pose Reset
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
#include "youbot_arm_model/youbot_joints.h"

#include "std_srvs/Empty.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/SetBool.h"

bool run = false;
bool running = false;

#define LOGGING
#ifdef LOGGING
	#include <fstream>
	using namespace std;
	ofstream logFile;
	ros::Time last;
	ros::Time start;
#endif



// Publisher für die Gelenkwinkel
ros::Publisher pub_joint;
// Publisher für die Geschwindigkeit der Plattform
ros::Publisher pub_base;

ros::ServiceClient reset_client;

void stopMovement(){
	geometry_msgs::Twist msg_base;

	msg_base.linear.x  = 0.0f;
	msg_base.linear.y  = 0.0f;
	msg_base.linear.z  = 0.0f;
	msg_base.angular.x = 0.0f;
	msg_base.angular.y = 0.0f;
	msg_base.angular.z = 0.0f; 

	pub_base.publish(msg_base); 
/*	
	brics_actuator::JointPositions msg_joints;
	ros::Time time_now = ros::Time::now();
	for( int i = 0; i <= 4; i++ ){
		brics_actuator::JointValue jv;
		jv.timeStamp = time_now;        
		jv.joint_uri = joint_names[i];
		jv.unit = "rad";
		jv.value = joint_offsets[i];	// alles zu 0 setzen

		msg_joints.positions.push_back(jv);
	}
	brics_actuator::Poison poison;
	poison.originator   = "";   // what?
	poison.description  = "";   // what?
	poison.qos          = 1.0f; // right?
	msg_joints.poisonStamp = poison;

	pub_joint.publish( msg_joints );
*/
}

void callback_rc_signal( const sensor_msgs::Joy::Ptr& msg )
{     
	if( msg->buttons[0]){
		run = true;
		if( !running )
			ROS_INFO("START");
	}
	if( msg->buttons[1]){
		run = false;	
		if( running ){
			ROS_INFO("STOP");
			stopMovement();
		}
	}	
	if( msg->buttons[3] ){
		std_srvs::Empty srv;
		reset_client.call(srv);
		// vielleicht hier auch Regler zurücksetzen
		ROS_INFO("Reset to Save Position + Disable all Movement");
		stopMovement();
		run = false;
	}
}

int main( int argc, char * argv[] ){

	ros::init(argc, argv, "Sim_Control");
	ros::NodeHandle nh("Sim_Control");

	ROS_INFO("Starte Simulationskontrolle");

	reset_client = nh.serviceClient<std_srvs::Empty>("/Dynamics/dynamics_resetPosition");

	ros::ServiceClient dynamics_client = nh.serviceClient<std_srvs::SetBool>("/Dynamics/dynamics_prop");
	ros::ServiceClient control_client = nh.serviceClient<std_srvs::SetBool>("/Controller/controller_prop");

	ros::Subscriber sub_Signal = nh.subscribe( "/joy", 100, callback_rc_signal);

	pub_joint = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 100);
	pub_base  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	#ifdef LOGGING
		char filePathName[] = "/home/youbot/Desktop/logDT.txt";
		logFile.open(filePathName); 
		if(!logFile.is_open()){
			ROS_ERROR("Logfile: '%s' konnte nicht geöffnet werden. Beende.", filePathName);
			return 0;
		}
		logFile << " Zeit , DT" << std::endl; 
		last = ros::Time::now();
		start = ros::Time::now();
	#endif

	ros::Rate loop_rate(200);

	while(ros::ok())
	{	
		std_srvs::SetBool srv;
		if( run ){
			if( !running ){
				srv.request.data = true;
				running = true;
			}else{
				srv.request.data = false;				
			}			
			#ifdef LOGGING
				ros::Time now = ros::Time::now();
				logFile << (now-start).toSec() << " , " << (now - last).toSec() << std::endl;
				last = now;
			#endif
			dynamics_client.call(srv);
      control_client.call(srv);
		}else{
			running = false;
		}		      	
		ros::spinOnce();
		loop_rate.sleep();
  }

  ros::spin();
  return 0;
} 
