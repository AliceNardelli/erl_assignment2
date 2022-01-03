#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "erl2/ArmorInterface.h"
#include <erl2/CheckAction.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <rosplan_action_interface/RPActionInterface.h>
#include "erl2/perceivehints.h"
#include <motion_plan/PlanningAction.h>


namespace KCL_rosplan {

PerceiveHintsActionInterface::PerceiveHintsActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	ros::ServiceClient client = nh.serviceClient<erl2::ArmorInterface>("/armor_interface");

}
bool PerceiveHintsActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
	erl2::ArmorInterface srv;
	srv.request.mode = 3;
        std::cout<<srv.request<<std::endl;
	this->client.call(srv);
	std::cout<<srv.response<<std::endl;
	if(srv.response.success==false){
	       return false;
	}
	    

	return true;
}
}


int main(int argc, char **argv) {

ros::init(argc, argv, "perceive_hint_action", ros::init_options::AnonymousName);

ros::NodeHandle nh("~");


KCL_rosplan::PerceiveHintsActionInterface my_aci(nh);

my_aci.runActionInterface();

return 0;
}


