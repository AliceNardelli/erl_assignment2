#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/ArmorInterface.h>
#include <erl2/CheckAction.h>
#include "erl2/checkconsistency.h"
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <rosplan_action_interface/RPActionInterface.h>
#include <motion_plan/PlanningAction.h>


namespace KCL_rosplan {

CheckConsistencyActionInterface::CheckConsistencyActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	ros::ServiceClient client = nh.serviceClient<erl2::ArmorInterface>("armor_interface");

}
bool CheckConsistencyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
	erl2::ArmorInterface srv;
	srv.request.mode = 2;

	if (client.call(srv))
	{
	    if(srv.response.success==false){
	       ROS_INFO("no new consistent hypotesis to check");
	       ROS_INFO("Action (%s) performed: not completed!", msg->name.c_str());
	       return false;
	    }
	    
	}
	else
	{
	    ROS_ERROR("Failed to call service armor_interface");
	    return false;
	}

	ROS_INFO("%d new consistent hypotesis to check!",srv.response.ID);
	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
}
}


int main(int argc, char **argv) {

ros::init(argc, argv, "check_concistency_action", ros::init_options::AnonymousName);

ros::NodeHandle nh("~");


KCL_rosplan::CheckConsistencyActionInterface my_aci(nh);

my_aci.runActionInterface();

return 0;
}


