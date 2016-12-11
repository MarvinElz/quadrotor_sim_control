#include "ros/ros.h"
#include "std_srvs/Empty.h"

int main( int argc, char * argv[] ){

   ros::init(argc, argv, "Sim_Control");
   ros::NodeHandle nh("Sim_Control");

   ros::ServiceClient dynamics_client = nh.serviceClient<std_srvs::Empty>("/Dynamics/dynamics_prop");
   ros::ServiceClient control_client = nh.serviceClient<std_srvs::Empty>("/Controller/controller_prop");

   ros::Rate loop_rate(50);

   while(ros::ok())
   {
      std_srvs::Empty srv;
      dynamics_client.call(srv);
      control_client.call(srv);
      ros::spinOnce();
      loop_rate.sleep();
   }

   ros::spin();
   return 0;
} 
