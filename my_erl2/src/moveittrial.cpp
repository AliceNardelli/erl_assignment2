#include <unistd.h>
#include <ros/ros.h>
#include <rosplan_action_interface/RPActionInterface.h>
#include "my_erl2/grippermotion.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "geometry_msgs/Pose.h"
#include <motion_plan/PlanningAction.h>
#include "my_erl2/ErlOracle.h"
#include "std_srvs/Trigger.h"
int state=0;
bool finished=false;
bool moved=false;
bool gripper=false;
bool first_turn_ended=false;
bool new_msg=false;
int up[4]={0,0,0,0};
int down[4]={0,0,0,0};
bool activate=false;
void Callback(const my_erl2::ErlOracle::ConstPtr& msg)
{
  std::cout<<"NEW"<<std::endl;
  std::cout<<msg->ID<<std::endl;
  std::cout<<msg->key<<std::endl;
  std::cout<<msg->value<<std::endl;
  new_msg=true;
}

void move(double x, double y,double t){

     		
	actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("reaching_goal", true);
	motion_plan::PlanningGoal goal;
	ac.waitForServer();
		
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.orientation.w = t;


	ac.sendGoal(goal);
	ac.waitForResult();
        ROS_INFO("MOVED");
        moved=true;
}
	
void move_gripper(double x, double y,bool start){
         robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	 const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
	 moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
	 kinematic_state->setToDefaultValues();
	 const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
	 moveit::planning_interface::MoveGroupInterface group("arm");
	 const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
	 if (start==true){
	  //group.setNamedTarget("zero");
	  //group.move(); 
	  std::cout<<"searching"<<std::endl;
	 }
	 else{
         
	  geometry_msgs::Pose pose1;
	  std::vector<double> joint_values;
	  double timeout = 0.1;	
          bool found_ik;
          moveit::planning_interface::MoveGroupInterface::Plan my_plan;
          for(int i=0;i<1;i++){
          
	  pose1.orientation.w = 0.70;
	  pose1.orientation.x = -0.00;
	  pose1.orientation.y = 0.00;
	  pose1.orientation.z = -0.71;
	  pose1.position.x =  x;
	  pose1.position.y =  y;
	  
	  if(first_turn_ended==false)	pose1.position.z = 0.70;
	  else{ 
	  pose1.position.z =  0.75;
	  std::cout<<"0.75"<<std::endl;
	  }
	  
	  group.setStartStateToCurrentState();
	  group.setApproximateJointValueTarget(pose1, "arm_link_04");

	  found_ik = kinematic_state->setFromIK(joint_model_group, pose1, timeout);

	  // Now, we can print out the IK solution (if found):
	  if (found_ik)
	  {
	    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	    for (std::size_t i = 0; i < joint_names.size(); ++i)
	    {
	      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	    }
	  }
	  else
	  {
	    ROS_INFO("Did not find IK solution");
	  }
	  
	  
	  group.setJointValueTarget(joint_values);
	  group.setStartStateToCurrentState();
	  group.setGoalOrientationTolerance(0.01);
	  group.setGoalPositionTolerance(0.01);

	  // Plan and execute
	  
	  group.plan(my_plan); 
	  group.execute(my_plan);
	  
	  std::cout << "Position 1 -> IK + setJointValue" << std::endl;
	  /*group.setNamedTarget("zero");
	  group.move();  
          std::cout << "Position 2 -> Zero" << std::endl;*/
          if(new_msg==true)
          {
          std::cout << "Location found" << std::endl;
          new_msg=false;
          //std::cout<<i<<""<<state<<std::endl;
          if(i==0){
             if (state==0){up[0]=1;ros::param::set("/wp1", 1);}
             else if (state==1) {up[1]=1;ros::param::set("/wp2", 1);}
             else if (state==2) {up[2]=1;ros::param::set("/wp3", 1);}
             else {up[3]=1;ros::param::set("/wp4", 1);}
          }
          else{
             if (state==0) down[0]=1;
             else if (state==1) down[1]=1;
             else if (state==2) down[2]=1;
             else down[3]=1;
          }
          }
          /*if(i==0) first_turn_ended=true;
          else first_turn_ended=false;*/
          }
	  
	  move(0,0,0);
	  state=state+1;
	  moved=false;
	  if (state==4){
           activate=false;
           finished=true;
           ros::param::set("/start", 0);
           std::cout<<up[0]<<up[1]<<up[2]<<up[3]<<std::endl;
           std::cout<<down[0]<<down[1]<<down[2]<<down[3]<<std::endl;
	  }}

}
void fsm(){
        if (finished==false){
        if( state==0 ){
        if (moved==false ){
	move(2.5,0,0);
	gripper=false;
	}
	else{
	if(gripper==false){
	std::cout<<"ONCE"<<std::endl;
        move_gripper(0.5,0,false);
        gripper=true;}
	}
	}
	else if( state==1 ){
	if (moved==false ){	
	move(-2.5,0,3.14);
	gripper=false;}
	else{
	if(gripper==false){
	std::cout<<"ONCE"<<std::endl;	
	move_gripper(0.5,0,false);
	gripper=true;}
	}
	}
	else if( state==2 ){
	if (moved==false ){
	move(0,2.6,3.14/2);
	gripper=false;}
	else{	
	if(gripper==false){
	std::cout<<"ONCE"<<std::endl;
	move_gripper(0.5,0,false);
	gripper=true;}
        }
	}
	else{
	if (moved==false ){
	move(0,-2.6,-3.14/2);
	gripper=false;}
	else{
	if(gripper==false){
	std::cout<<"ONCE"<<std::endl;		
	move_gripper(0.5,0,false);
	gripper=true;}}
	}
	}
}
bool srv_clbk(std_srvs::Trigger::Request  &req,
         std_srvs::Trigger::Response &res)
{
   activate=true;
   //while(activate==true) fsm();
   return true;
}
int main(int argc, char **argv) {

	ros::init(argc, argv, "gripper_motion", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");	
	ros::AsyncSpinner spinner(1);
	spinner.start();
	sleep(2.0);
	ros::ServiceServer service = nh.advertiseService("/init_service", srv_clbk);
	ros::Subscriber sub = nh.subscribe("/oracle_hint", 1000, Callback);       
        move_gripper(0, 0,true);
        ros::Rate loop_rate(10);
        while (ros::ok()&& finished==false){
                if(activate==true) fsm();
                ros::spinOnce();
                loop_rate.sleep();
        }
	return 0;
}

