#include <unistd.h>
#include <ros/ros.h>
#include <rosplan_action_interface/RPActionInterface.h>
#include "erl2/move.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

int actual_x;
int actual_y;
ros::Publisher pub;

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg){
    
    actual_x=msg->pose.pose.position.x;
    actual_y=msg->pose.pose.position.y;    
}
int distance(int x,int y){
      return sqrt(x^2 + y^2);
}

namespace KCL_rosplan {
	
MoveActionInterface::MoveActionInterface(ros::NodeHandle &nh) {
// here the initialization
}



bool MoveActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
                
                
		std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		
		actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("reaching_goal", true);
		motion_plan::PlanningGoal goal;
		ac.waitForServer();
		if(msg->parameters[2].value == "wp1"){
		goal.target_pose.pose.position.x = 2.5;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[2].value == "wp2"){
		goal.target_pose.pose.position.x = -2.5;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[2].value == "wp3"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 2.6;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[2].value == "wp4"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = -2.6;
		goal.target_pose.pose.orientation.w = 0.0;
		}

		ac.sendGoal(goal);
		ac.waitForResult();
		/*geometry_msgs::Twist vel;
		int des_x=0;
		int des_y=0;
                if(msg->parameters[2].value == "wp1") des_x=1;
		else if (msg->parameters[2].value == "wp2")des_x=-1;
		else if (msg->parameters[2].value == "wp3")des_y=1;
		else if (msg->parameters[2].value == "wp4")des_y=-1;
                //compute the distance on x and on y between actual position and target
                //d_x and d_y are vector that go from actual position to the target one
                int d_x=des_x-actual_x;
                int d_y=des_y-actual_y;

                //if the distance is smaller than 0.1 I ask for a new target 
                //in the other case I set velocity to get the target
               int d=distance(d_x,d_y);

               while(d>0.1){
      
       
                    //set velocity
                    //direction of velocity is given by d_x and d_y
                    vel.linear.x=10*d_x;  
                    vel.linear.y=10*d_y;

                    //publish velocity
                    pub.publish(vel);
                    //show status of the robot
                    
                    d_x= des_x-actual_x; 
                    d_y=des_y-actual_y;
                    d=distance(d_x,d_y);
               }*/
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		
		return true;

}
}


int main(int argc, char **argv) {

ros::init(argc, argv, "rosplan_interface_move", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
ros::Subscriber sub=nh.subscribe("/odom",1000,positionCallback);
pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
KCL_rosplan::MoveActionInterface move(nh);
move.runActionInterface();
return 0;
}

