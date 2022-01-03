#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/ArmorInterface.h>
#include <erl2/CheckAction.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <rosplan_action_interface/RPActionInterface.h>
#include "erl2/checkcorrect.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <sstream>

namespace KCL_rosplan {

CheckCorrectActionInterface::CheckCorrectActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	ros::ServiceClient client = nh.serviceClient<erl2::ArmorInterface>("armor_interface");

}
bool CheckCorrectActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action
	erl2::ArmorInterface srv;
	
	std::array<std::string, 3> curr_hypo;
	//ros::param::get("/current_hypotesis",curr_hypo);
	//n.getParam("/current_hypotesis",curr_hypo)
	//PRENDERE I PARAM UNO PER UNO....MERDA DI CPP
	srv.request.mode = 1;
	srv.request.ID=stoi(curr_hypo[0]);

	if (client.call(srv))
	{
	    if(srv.response.success==false){
	       ROS_INFO("%d incorrect hypotesis",srv.response.ID);
	       ROS_INFO("Action (%s) performed: not completed!", msg->name.c_str());
	       return false;
	    }
	    
	}
	else
	{
	    ROS_ERROR("Failed to call service armor_interface");
	    return false;
	}

	ROS_INFO("%d correct hypotesis! The game is ended.",srv.response.ID);
	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
}
}


int main(int argc, char **argv) {

ros::init(argc, argv, "check_correct_action", ros::init_options::AnonymousName);

ros::NodeHandle nh("~");


KCL_rosplan::CheckCorrectActionInterface check(nh);

check.runActionInterface();

return 0;
}


