#! /usr/bin/env python
"""
Module:
	Reasoning
Author:
	Alice Nardelli alice.nardelli98@gmail.com
ROS nodes that implement the master of the Software Architecture and menages the whole simulation
Service Client :
	/armor_interface to load the ontology
	/init_service to start the initial phase
	/rosplan_problem_interface/problem_generation_server to load the problem
	/rosplan_planner_interface/planning_server to generate the plan
	/rosplan_parsing_interface/parse_plan to parse the plan
	/rosplan_plan_dispatcher/dispatch_plan to dispatch the actions
"""
import sys
import rospy
import actionlib
import my_erl2.msg
import math
import time
from my_erl2.srv import ArmorInterface, ArmorInterfaceRequest
from std_srvs.srv import Empty, Trigger, TriggerResponse
from rosplan_dispatch_msgs.srv import DispatchService

def query_planner():
    '''
            Description of the query function:
            This function is used to interface with ROSPlan. It continously load the problem depending on the actual state grounded predicates, generate a plan, parse it and dispatch actions.
            The function is runned each time an action fails since the goal of the problem is not reached.
       
    ''' 
    global client_armor_interface,client_pb,client_plan,client_dsp,client_parse,client_init
    global dsp
    
    if dsp==True:
	   
	    resp_pb=client_pb()
	    resp_plan=client_plan()
	    resp_parse=client_parse()
	    #while dsp==True:
	    resp_dsp=client_dsp()
	    print(resp_dsp) 
	    if(resp_dsp.goal_achieved==False):
	      dsp=True
	    else:
	      dsp=False 
	      print('GAME ENDED')                           

    
def main():
    '''
            Description of the main function:
            In the main function firstly the ontology is loaded, then call the /init_service to start initial behavior then starts to interface with ROSPlan.
       
    ''' 
    global client_armor_interface,client_pb,client_plan,client_dsp,client_parse,client_init
    global dsp
    # init node
    rospy.init_node('reasoning')
    #init ros clients
    client_armor_interface = rospy.ServiceProxy('/armor_interface', ArmorInterface)
    client_pb = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
    client_plan = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
    client_parse=rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    client_dsp=rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
    client_init=rospy.ServiceProxy('/init_service',Trigger)
    #load the ontology
    rospy.wait_for_service('/armor_interface')
    msg=ArmorInterfaceRequest()
    msg.mode=0
    resp=client_armor_interface(msg)
    rospy.wait_for_service('/init_service')
    dsp=True
    #init the initial state
    resp=client_init()
    rate = rospy.Rate(10)
    while dsp==True:
        s=rospy.get_param("/start")
        if(s==0):
           query_planner()
        rate.sleep()


if __name__ == '__main__':
    main()

