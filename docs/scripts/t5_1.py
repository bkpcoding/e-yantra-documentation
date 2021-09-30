#! /usr/bin/env python
#sorting
import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import sys
import copy
import json
import requests
import time

import threading

from pkg_task4.msg import msgFile
from hrwros_gazebo.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from hrwros_gazebo.msg import Model
from  pkg_vb_sim.srv import vacuumGripper
from moveit_commander import MoveGroupCommander, roscpp_initialize
from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgMqttSub
import datetime

colour = []							#lsit storing the colour of packages
i=0
j=0
pack = []
res = dict()
order=[]
n = 0							#list storing package name
class sort:
    """
    This class will sort the packages according to their colour

    """

    # Constructor
    def __init__(self):
        """
        creates the variables associated with the class

        """

        rospy.init_node('node_t4_1', anonymous=True)

        self._robot_ns = '/' + 'ur5_2'
        self._planning_group = "ur5_2_planning_group"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')
        self.pow_belt(90)


    # Destructor
    def __del__(self):
        """
        Destructor
        To be called at the end of the instance

        """

        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


    def go_to_predefined_pose(self, arg_pose_name):
        """
        Function to move the arm to one of the predifined poses
        :type name: string
        :param name: position name to which the arm must move to

        """

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        """
        Function to set the joint angles of the arm to the values given
        as arguments

        :type name: list
        :param name: the numerical values for the joint angles to be set

        """

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def callback(self,msg):
        """
        function which is called whenever a mesaage is recieved
        :type name : string
        :param name: message containing the positons of the box under the logical camera

        """

        try:						#logical camera call back function
	    global pack
	    global j
	    #print("**********************************")
	    print(len(msg.models))
	    print(msg.models[0].type)
	    print(msg.models[1].type)

	    if(msg.models[1].type == pack[j]):					#checks for the package under logical camera same as package name in pack list
	        if(msg.models[1].pose.position.y<0.05 and msg.models[1].pose.position.y>-0.05):	#check the coordinate of package
		    j=j+1
		    self.pow_belt(0)
		    self.belt_stop()
	except:
	    pass

    def get_time_str(self):
        """
        Get the timestamp of the time of starting
        """
    	timestamp = int(time.time())
    	value = datetime.datetime.fromtimestamp(timestamp)
    	str_time = value.strftime('%m/%d/%Y %H:%M:%S')
   	return str_time

    def belt_stop(self):
        """
        Sort the packages based on the color detected by the logical_camera_2

        """

        global colour
        global i
	    col = colour[i]
        self.vacuum_griper(True)
	    rospy.sleep(0.5)
        print ("activated the vacuum gripper")
	#home_pose
        lst_joint_angles =[0.06949,-2.35,-1.187,-1.23,1.5436,-0.02]
	#green
        lst_joint_angles_green = [1.667,-2.1484,-0.7726,-1.751,1.56,0.096]

        lst_joint_angles_up = [0,-2.383,-0.607,-1.721,1.61096,-1.57]

	#red
        lst_joint_angles_red = [1.458306,-1.034272,1.1572377,-1.723733,-1.567390,3.0291449]



        print("Going to the respective bin !")

        print("The colour is - ")
        print(col)
	    self.set_joint_angles(lst_joint_angles_up)
        if(col == "red"):
            print("ur5_2 going to red bin")
            res=self.set_joint_angles(lst_joint_angles_red)
	    if(res == False):
		    self.go_to_predefined_pose("allZeros")
		    self.set_joint_angles(lst_joint_angles_red)


        elif(col == "green"):
            print("ur5_2 going to green bin")
            res=self.set_joint_angles(lst_joint_angles_green)
	    if(res == False):
	        self.go_to_predefined_pose("allZeros")
		    self.set_joint_angles(lst_joint_angles_green)



        elif(col == "yellow"):
            print("ur5_2 going to yellow bin")
            self.go_to_predefined_pose("allZeros")

        self.pow_belt(100)
        self.vacuum_griper(False)
        print("Deacivated the vacuum_gripper !")
	    self.shipped()

        res2 = self.set_joint_angles(lst_joint_angles)
        if(res2==False):
	        self.pow_belt(0)
	        self.go_to_predefined_pose("allZeros")
	        self.set_joint_angles(lst_joint_angles)
	        self.pow_belt(100)
            i=i+1

	#fnction to power the belt
    def pow_belt(self,power):
        """
        This function stops the conveyorBelt
        :type name: int
        :param name: specifies the operating power of the conveyer belt from range 11 to 100

        """
        conveyor_belt=rospy.ServiceProxy("/eyrc/vb/conveyor/set_power",conveyorBeltPowerMsg)
        conveyor_belt(power)

	#function to activate or deactivate the vaccum gripper
    def vacuum_griper(self,state):
        """
        Activates the Vaccum Gripper
        :type name: string
        :param name: the state of the vaccum gripper i.e.. activated or deactivated

        """
        vaccum_gripper=rospy.ServiceProxy("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2",vacuumGripper)
	    vaccum_gripper(state)
	    print(state)

	#call back function of the  subscriber

    def func_callback_topic_my_topic(self,myMsg):
        """
        Callback function for the subscriber ur5_2
        :type name: list
        :param name: contains the list of parameters to detect the package with the input coming from the logical_camera_2

        """

        global colour
	global pack
        rospy.loginfo("Data Received: (%s)", myMsg.col)				#store the colour of the package recieved on the mesaage file from node_t4 in  list
        colour.append(myMsg.col)
	    rospy.loginfo("Data Received: (%s)", myMsg.name)			#store the name of the package recieved on the mesaage file from node_t4 in  list
    	pack.append(myMsg.name)

    def shipped(self):
        """
        Function to send the shipped goods to the Google sheets
        """
        global order
        global n
    	time = get_time_str()
    	if(order[n]['item'] == 'Medicine'):
            priority = "HP"
            cost = 450
    	elif(order[n]['item'] =='Clothes' ):
            priority = "LP"
            cost = 150
    	elif(order[n]['item'] == 'Food'):
            priority = "MP"
            cost = 250


    	parameters = {"id":"OrdersDispatched", "Team Id":"#VB0865", "Unique Id": "aBCaRvEc", "Order ID":order[n]['order_id'],"City":order[n]['city'], "Item":order[n]['item'],"Priority":priority, "Dispatch Quantity":"1","Cost":cost, "Dispatch Status":"YES", "Dispatch Date and Time":time}

    	URL = "https://script.google.com/macros/s/AKfycbx4_KSB3dGiZOL_ExE1DJ_x1RrLXo2-onAlmf9jL6Ksmshvnuael-y6mw/exec"

    	response = requests.get(URL, params=parameters)

    	print(response.content)
    	n = n+1

def start(my_msg):
    """
    Function to get the receive orders from google sheets through message file
    :type name: string
    :param name: contains the columns of the google sheet of the order recieved

    """
    global res
    global order
    print("The message is : ")

    print('_____________________')

    res = json.loads(my_msg.message)
    order.append(res)

def main():
   """
   The main Function
   """
    ur5_2 = sort()

    rospy.Subscriber("my_topic", msgFile, ur5_2.func_callback_topic_my_topic) 	#subscribing to my_topic to get colour and name of the package
    rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,ur5_2.callback)
    rospy.Subscriber("/ros_iot_bridge/mqtt/sub",msgMqttSub,start)
    lst_joint_angles = [0.06949,-2.35,-1.187,-1.23,1.5436,-0.02] 			#conveyor belt pose

    ur5_2.set_joint_angles(lst_joint_angles)

    rospy.spin()


    del ur5_2


if __name__ == '__main__':

    main()
