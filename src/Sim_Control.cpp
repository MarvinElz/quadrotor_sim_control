#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/SetBool.h"

bool run = true;
bool running = false;

void callback_rc_signal( const sensor_msgs::Joy::Ptr& msg )
{     
   if( msg->buttons[0]){
	run = true;
	if( !running )
		ROS_INFO("START");
   }
   if( msg->buttons[1]){
	run = false;	
	if( running )
		ROS_INFO("STOP");
   }	
}

int main( int argc, char * argv[] ){

   ros::init(argc, argv, "Sim_Control");
   ros::NodeHandle nh("Sim_Control");

   ros::ServiceClient dynamics_client = nh.serviceClient<std_srvs::SetBool>("/Dynamics/dynamics_prop");
   ros::ServiceClient control_client = nh.serviceClient<std_srvs::SetBool>("/Controller/controller_prop");

   ros::Subscriber sub_Signal = nh.subscribe( "/joy", 100, callback_rc_signal);

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
