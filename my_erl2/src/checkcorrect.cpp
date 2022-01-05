#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_erl2/ArmorInterface.h>
#include <my_erl2/CheckAction.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <rosplan_action_interface/RPActionInterface.h>
#include "my_erl2/checkcorrect.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <sstream>
ros::ServiceClient client;
namespace KCL_rosplan {

CheckCorrectActionInterface::CheckCorrectActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	//ros::ServiceClient client = nh.serviceClient<my_erl2::ArmorInterface>("armor_interface");

}
bool CheckCorrectActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action
	my_erl2::ArmorInterface srv;
	
	int curr_hypo;
	//ros::param::get("/currID",curr_hypo);
	//n.getParam("/current_hypotesis",curr_hypo)
	//PRENDERE I PARAM UNO PER UNO....MERDA DI CPP
	srv.request.mode = 1;
	//srv.request.ID=curr_hypo;
        //std::cout<<srv.request.ID<<std::endl;
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

client = nh.serviceClient<my_erl2::ArmorInterface>("/armor_interface");
KCL_rosplan::CheckCorrectActionInterface check(nh);

check.runActionInterface();

return 0;
}


