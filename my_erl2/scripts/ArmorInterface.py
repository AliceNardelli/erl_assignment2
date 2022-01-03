#! /usr/bin/env python
"""
Module:
	AnnounceHypotesis
Author:
	Alice Nardelli alice.nardelli98@gmail.com
ROS nodes used for simulating the robot announcement. Given an hypotesis it announces it simply printing on terminal.
Service :
	/announce_service to get the hypotesis to announce
"""
import sys
import rospy
import actionlib
import erl2.msg
from erl2.msg import ErlOracle
import math
import time
from erl2.srv import Oracle, Announcement, ArmorInterface, ArmorInterfaceResponse, AnnouncementRequest
from armor_msgs.srv import ArmorDirective
# mode 0 initialization
# mode 1 perception
# mode 2 consistency


def ontology_interaction(
        command,
        primary_command_spec,
        secondary_command_spec,
        arg):
    '''
           Description of the ontology_interaction function:
           This function is used to interface with the ontology through the aRMOR action server. It get as argument all fields of the ArmorDirective message to fill.
           Args:
              command
              primary_command_spec
              secondary_command_spec
              arg
           Returns:
              msg(ArmorDirectiveRes)

    '''
    global client_armor
    rospy.wait_for_service('armor_interface_srv')

    msg = ArmorDirective()
    msg.client_name = 'tutorial'
    msg.reference_name = 'ontoTest'
    msg.command = command
    msg.primary_command_spec = primary_command_spec
    msg.secondary_command_spec = secondary_command_spec
    msg.args = arg
    print('send req')
    resp = client_armor(msg)
    print('rec res')
    return resp


def menage_response(st):
    '''
             Description of the menage_response function:
             This function is used to menage the strings retrieved by aRMOR service
             Args:
               st: string to manage
             Returns:
               st: arranged string

    '''
    st = st.replace("<http://www.emarolab.it/cluedo-ontology#", "")
    st = st.replace(">", "")

    return st


def load_initialize_ontology():
    # load ontology
    ontology = rospy.get_param('ontology')
    ontology_path = rospy.get_param('ontology_path')

    r1 = ontology_interaction(
        'LOAD', 'FILE', '', [
            ontology_path, ontology, 'true', 'PELLET', 'true'])
    # add class incorrect

    r2 = ontology_interaction(
        'ADD', 'CLASS', 'CLASS', [
            'INCORRECT', 'HIPOTESIS'])
    # set all classes as mutually disjoint
    r3 = ontology_interaction('DISJOINT', 'CLASS', '', ['PERSON', 'PLACE'])
    r4 = ontology_interaction('DISJOINT', 'CLASS', '', ['PLACE', 'WEAPON'])
    r5 = ontology_interaction('DISJOINT', 'CLASS', '', ['WEAPON', 'PERSON'])
    r6 = ontology_interaction('DISJOINT', 'CLASS', '', ['INCORRECT', 'PERSON'])
    r7 = ontology_interaction('DISJOINT', 'CLASS', '', ['WEAPON', 'INCORRECT'])
    r8 = ontology_interaction('DISJOINT', 'CLASS', '', ['INCORRECT', 'PLACE'])


def clbk(req):
    '''
    Description of the callback:
    This function retrieves the goal of the custom MoveAction. Inside the goal are specified the actual and the goal position of the robot and the name of the destination room.
    Movement is simulate as a wait procedure. The wait is proportionale to the length of the path. The path is a straigth line between the two points.
    The action continously provides as feedbact actual (x,y,yaw) of the robot.
    Args:
       msg(MoveGoal): goal retrieved by */move_action_server/goal* topic
    Returns:
       msg(MoveFeedback): actual (x,y,yaw) of the robot published on */move_action_server/feedback* topic
       msg(MoveResult): true

    '''
    global client_oracle_solution, client_armor, client_announce
    global erloracle
    _res = ArmorInterfaceResponse()

    # initialisation of the ontology
    if req.mode == 0:
        rospy.loginfo('Initialization')
        load_initialize_ontology()
        _res.mode = 0
        _res.success = True
        return _res
    if req.mode == 1:
        rospy.loginfo('Check correct')
        # get correct solution
        rospy.wait_for_service('oracle_solution')
        resp = client_oracle_solution()
        _res.mode = 1
        # announce the hypotesis
        rospy.wait_for_service('announce_service')
        current_hypotesis = rospy.get_param('current_hypotesis')
        msg = AnnouncementRequest()
        msg.who = current_hypotesis[1]
        msg.where = current_hypotesis[2]
        msg.what = current_hypotesis[3]
        a = client_announce(msg)
        print(req.ID)
        if resp.ID == req.ID:
            print('cooreect')
            resp=ontology_interaction('SAVE','INFERENCE','',['/root/ros_ws/src/exprob_ass1/cluedo_ontology_inference.owl'])
            _res.success = True
            _res.ID = req.ID
        else:
            print('not correct')
            _res.success = False
            _res.ID = req.ID
            r1 = ontology_interaction('REMOVE', 'IND', '', [str(req.ID)])
            r3 = ontology_interaction(
                'ADD', 'IND', 'CLASS', [
                    str(req.ID), 'INCORRECT'])
            r2 = ontology_interaction('REASON', '', '', [])
        return _res
    if req.mode == 2:
        resp2 = ontology_interaction('DISJOINT', 'IND', 'CLASS', ['PERSON'])
        resp2 = ontology_interaction('DISJOINT', 'IND', 'CLASS', ['PLACE'])
        resp2 = ontology_interaction('DISJOINT', 'IND', 'CLASS', ['WEAPON'])
        # ask for complete hypotesis
        resp_c = ontology_interaction('QUERY', 'IND', 'CLASS', ['COMPLETED'])

        # ask for incostintent hypotesis
        resp_i = ontology_interaction(
            'QUERY', 'IND', 'CLASS', ['INCONSISTENT'])

        _res.mode = 2

        # if the length is equal means that there is not consistent
        # hypotesis to check
        if len(
                resp_i.armor_response.queried_objects) == len(
                resp_c.armor_response.queried_objects):

            _res.success = False

            rospy.loginfo('no new consistent hypotesis')
        else:
            _res.success = True
            rospy.loginfo('Check consistent')
            complete = []

            for i in range(len(resp_c.armor_response.queried_objects)):

                st = menage_response(resp_c.armor_response.queried_objects[i])
                complete.append(st)

            if len(resp_i.armor_response.queried_objects) > 0:
                for i in range(len(resp_i.armor_response.queried_objects)):
                    st = menage_response(
                        resp_i.armor_response.queried_objects[i])
                    complete.remove(st)
            # if needed here the code to add the consistent hypo on param
            # server
            _res.ID = complete[0]
            consistent_who = ontology_interaction(
                'QUERY', 'OBJECTPROP', 'IND', ['who', complete[0]])

            who = menage_response(
                consistent_who.armor_response.queried_objects[0])
            consistent_where = ontology_interaction(
                'QUERY', 'OBJECTPROP', 'IND', ['where', complete[0]])
            where = menage_response(
                consistent_where.armor_response.queried_objects[0])
            consistent_what = ontology_interaction(
                'QUERY', 'OBJECTPROP', 'IND', ['what', complete[0]])
            what = menage_response(
                consistent_what.armor_response.queried_objects[0])

            # store the consistent hypotesis in the parameter server
            rospy.set_param(
                'current_hypotesis', [
                    complete[0], who, where, what])
        return _res
    else:
        print("adding new obj")
        _res.mode = 3
        # read a message ErlOracle
        #erloracle = rospy.wait_for_message('/oracle_hint', ErlOracle)
        if erloracle.key == '' or erloracle.value == '' or erloracle.ID == '':
            rospy.loginfo('malformed hints hint perceived: lens is dirty!!')
            _res.success = False

        else:
            # ask the incorrect hypotesis collected
            print("query incorrect")
            r = ontology_interaction('QUERY', 'IND', 'CLASS', ['INCORRECT'])
            print("incorrect ended")
            incorrect = []
            for i in range(len(r.armor_response.queried_objects)):
                st = menage_response(r.armor_response.queried_objects[i])
                incorrect.append(st)

            # if the perceived object belong to an hypotesis already checked as
            # incorrect robot will discard it
            if erloracle.ID in incorrect:
                rospy.loginfo(
                    'The perceived hint has an id associated to an incorrect hypotesis: I will discard it')
                _res.success = False

            # otherwise it add the object to the ontology
            else:
                print("add hp")
                
                _res.success = True
                r1 = ontology_interaction(
                    'ADD', 'OBJECTPROP', 'IND', [
                        erloracle.key, str(erloracle.ID), erloracle.value])
                print("add obj")
                if erloracle.key == 'where':
                    resp = ontology_interaction(
                        'ADD', 'IND', 'CLASS', [
                           erloracle.value, 'PLACE'])
                elif erloracle.key == 'what':
                    resp = ontology_interaction(
                        'ADD', 'IND', 'CLASS', [
                            erloracle.value, 'WEAPON'])
                else:
                    resp = ontology_interaction(
                        'ADD', 'IND', 'CLASS', [
                            erloracle.value, 'PERSON'])
                # disjoint all element of one classes is necessarily for having
                # multiple hints of a class belonging to the same hypotesis
                resp2 = ontology_interaction(
                    'DISJOINT', 'IND', 'CLASS', ['PERSON'])
                resp2 = ontology_interaction(
                    'DISJOINT', 'IND', 'CLASS', ['PLACE'])
                resp2 = ontology_interaction(
                    'DISJOINT', 'IND', 'CLASS', ['WEAPON'])
                # reason about the class
                r2 = ontology_interaction('REASON', '', '', [])
                rospy.loginfo('PERCEIVED: ['+str(erloracle.ID) + ':' + erloracle.value + ',' + erloracle.key + ']')
        return _res
        
def callback(data):
    #print(data)
    global erloracle
    erloracle.value=data.value
    erloracle.key=data.key
    erloracle.ID=data.ID
    

def main():
    global client_armor,client_oracle_solution,client_announce
    global erloracle
    # init node
    rospy.init_node('armor_interface_client')
    # init service
    srv = rospy.Service('/armor_interface', ArmorInterface, clbk)
    client_oracle_solution = rospy.ServiceProxy('oracle_solution', Oracle)
    client_armor = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    client_announce = rospy.ServiceProxy('announce_service', Announcement)
    rospy.Subscriber("/oracle_hint", ErlOracle, callback)
    erloracle=ErlOracle()
    erloracle.key=''
    erloracle.value=''
    erloracle.ID=''
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    main()
