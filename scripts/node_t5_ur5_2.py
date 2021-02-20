#! /usr/bin/env python
"""
Controls the UR5 near the bins to pick up boxes from the conveyor. 
Borrows methods from :mod:`lib_task5` to aid execution, and uses
multithreading to operate the UR5_2 and conveyor simultaneously.
"""

import os
import threading
import json
import datetime
from math import radians
import rospy
from rospy.exceptions import ROSInterruptException
import geometry_msgs.msg

from hrwros_gazebo.msg import LogicalCameraImage
from std_msgs.msg import String
from lib_task5 import Ur5Moveit  # The library for the UR5s
from node_iot_action_client import RosIotBridgeActionClient  # ROS-IoT Bridge Client


def env_data():
    """
    Data of all environment-specific parameters:

    1. Vacuum Gripper Width
    2. Box Size
    3. Home Position Joint angles for the UR5

    :return: All environment-specific data specified in the function.
    :rtype: list
    """
    box_length = 0.15  # Length of the box
    vacuum_gripper_width = 0.117  # Vacuum Gripper Width
    home_joint_angles = [radians(0),
                         radians(-120),
                         radians(-85),
                         radians(-65),
                         radians(90),
                         radians(0)]

    # Return data when called
    return [box_length,
            vacuum_gripper_width, home_joint_angles]


def pose_set(trans, rot):
    """
    Assigns pose values w.r.t the world-frame to a PoseStamped object.

    :arg list(float) trans: Translation Values.
    :arg list(float) rot: RPY Rotation Values.

    :returns: The complete pose with values.
    :rtype: PoseStamped object
    """
    # If you want to override any values, use this
    override = [trans, [-0.5, -0.5, 0.5, 0.5]]
    if override:
        trans = override[0]
        rot = override[1]

    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = trans[0]
    pose.pose.position.y = trans[1]
    pose.pose.position.z = trans[2]
    pose.pose.orientation.x = rot[0]
    pose.pose.orientation.y = rot[1]
    pose.pose.orientation.z = rot[2]
    pose.pose.orientation.w = rot[3]

    return pose


def box_plan(box_name, box_length, vacuum_gripper_width):
    """
    Pick-planning for the boxes.

    :arg str box_name: The name of the box as detected through the Logical Camera.
    :arg float box_length: The size of the box in metres.
    :arg float vacuum_gripper_width: The width of the vacuum gripper.
    """
    # Offset for end effector placement
    delta = vacuum_gripper_width + (box_length / 2)

    # Obtaining the TF transform of the box
    (box_trans, box_rot) = UR5.tf_listener.lookupTransform("/world",
                                                           "/logical_camera_2_%s_frame" % box_name,
                                                           rospy.Time(0))
    # Execute pick operation
    box_pose = pose_set(box_trans, box_rot)  # Collating pose values
    box_pose.pose.position.z = box_pose.pose.position.z + delta  # Adding Z Offset
    UR5.hard_go_to_pose(box_pose, 3)  # Executing pick travel
    # Activate Vacuum Gripper
    os.system\
(
    'rosservice call /eyrc/vb/ur5/activate_vacuum_gripper/ur5_2 "activate_vacuum_gripper: true"\n'
)

    # Travelling back to home
    UR5.hard_set_joint_angles(ENV_VALUES[2], 3)
    os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 100"')
    # Log the operation
    rospy.logwarn(
        "Package '%s' picked!" % box_name)


class UR5Two(object):
    """
    Class for the module.

    :var float package_pos: Stores the conveyor position of the detected box.
    :var str package_name: Stores models' names from :meth:`camera_callback`.
    :var list(str) package_colours: Stores package colours from :meth:`package_callback`.
    :var bool ready_flag: Start-stop variable for operation of the UR5_2.
    :var dict order: The current order to be shipped.
    :var rospy.Subscriber sub_package_colour: Subscriber to receive the list of colours
      from the Logical Camera.
    """
    def __init__(self):
        """
        Constructor containing essential data.
        """
        self.package_pos = 0  # Stores the conveyor position of the detected box
        self.package_name = ''  # String to store models' names from camera_callback()
        self.package_colours = []  # String list to store packages colours from package_callback()
        self.ready_flag = False  # Start-stop variable
        self.order = {}  # The current dispatched order to be shipped
        self.sub_package_colour = rospy.Subscriber('/package_colour',
                                                   String,
                                                   self.package_callback)

        rospy.Subscriber('/eyrc/vb/logical_camera_2',
                         LogicalCameraImage,
                         self.camera_callback)

        rospy.Subscriber('/dispatched_order', String, self.order_callback)


    def smart_stop(self):
        """
        Multithreading function for conveyor start-stop.
        """
        # Conveyor Signal variable
        power_flag = False
        while True:
            if ('packagen' in self.package_name) and (abs(self.package_pos) < 0.4):
                if power_flag:
                    os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 0"')
                self.ready_flag = True
                power_flag = False
            elif not power_flag:
                os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 100"')
                power_flag = True

    def camera_callback(self, msg_camera):
        """
        Callback function for Conveyor Logical Camera Subscriber

        :arg LogicalCameraImage msg_camera:
            Data about all the objects detected by the Logical Camera.
        """
        if msg_camera.models:
            if len(msg_camera.models) == 1:
                self.package_name = msg_camera.models[0].type
                self.package_pos = msg_camera.models[0].pose.position.y
            else:
                self.package_name = msg_camera.models[1].type
                self.package_pos = msg_camera.models[1].pose.position.y

    def order_callback(self, msg_order):
        """
        Callback function to receive the data of the currently dispatched order from UR5_1.

        :arg str msg_order: A JSON string dump of the order data dictionary.
        """
        self.order = json.loads(msg_order.data)

    def package_callback(self, msg_package_list):
        """
        Callback function for the package colour decoder Subscriber.

        :arg list(str) msg_package_list: CSV-formatted list of all the package colours in order.
        """

        temp_var = msg_package_list.data.split(',')  # Converting the CSV

        if len(temp_var) == 9 and 'NA' not in temp_var:
            rospy.logwarn("Configuration stored:")
            rospy.logwarn(temp_var)
            self.sub_package_colour.unregister()
            self.package_colours = temp_var

    def bin_plan(self):
        """
        Place-planning for the bins.
        """
        colour_code = ''  # Terminal Colour Code
        bin_name = ""

        # Obtain colour/priority of package
        if self.order["Priority"] == "HP":
            bin_name = "r"
            ship_time = datetime.timedelta(days=1)
            colour_code = '91'
        elif self.order["Priority"] == "MP":
            bin_name = "y"
            ship_time = datetime.timedelta(days=3)
            colour_code = '93'
        elif self.order["Priority"] == "LP":
            bin_name = "g"
            ship_time = datetime.timedelta(days=5)
            colour_code = '92'

        # Travel to bin
        UR5.moveit_play_planned_path_from_file(UR5.file_path +
                                               'ur5_2/', 'home_to_%s.yaml' % bin_name[0])
        # Deactivate the Gripper
        os.system\
(
    'rosservice call /eyrc/vb/ur5/activate_vacuum_gripper/ur5_2 "activate_vacuum_gripper: false"\n'
)

        # Log the operation
        if bin_name == 'r':
            colour_code = '91'
        elif bin_name == 'y':
            colour_code = '93'
        elif bin_name == 'g':
            colour_code = '92'
        rospy.loginfo(
            '\033[{}m{} Package with order ID {} shipped! \033[0m'.format(
                colour_code, self.order["Priority"],
                self.order["Order ID"]))

        ship_instant = datetime.datetime.now()

        # Update order shipping status
        order = {"id": "OrdersShipped",
                 "Team Id": "VB#1202",
                 "Unique Id": "isAmiTvb",
                 "Order ID": self.order["Order ID"],
                 "City": self.order["City"],
                 "Item": self.order["Item"],
                 "Priority": self.order["Priority"],
                 "Shipped Quantity": self.order["Dispatch Quantity"],
                 "Cost": self.order["Cost"],
                 "Shipped Status": "YES",
                 "Shipped Date and Time": ship_instant.strftime("%Y-%m-%d %H:%M:%S"),
                 "Estimated Time of Delivery": (ship_instant + ship_time).strftime("%d-%m-%Y")}

        # Updating Spreadsheet
        ACTION_CLIENT.send_goal_pls(order)

        # Travel to home
        UR5.moveit_play_planned_path_from_file(UR5.file_path + 'ur5_2/',
                                               '%s_to_home.yaml' % bin_name[0])


    def controller(self):
        """
        Executes the main operations, coordinating the UR5_2.
        """
        # Go to home position
        UR5.moveit_play_planned_path_from_file(UR5.file_path + 'ur5_2/', 'zero_to_home.yaml')

        # Execute planning
        while not rospy.is_shutdown():
            try:
                if ('packagen' in self.package_name) and self.ready_flag: # When box detected
                    box_plan(self.package_name, ENV_VALUES[0], ENV_VALUES[1])
                    self.bin_plan() # Execute place operation
                    self.package_colours.pop(0) # Removing item from list
                    self.ready_flag = False # Signalling conveyor

            except ROSInterruptException:
                quit()


if __name__ == '__main__':

    rospy.sleep(10)

    UR5 = Ur5Moveit("ur5_2")  # Initialise the UR5_2
    ACTION_CLIENT = RosIotBridgeActionClient()  # Start the ROS-IoT Bridge

    UR5_2 = UR5Two()

    # Start the separate conveyor control thread
    T = threading.Thread(target=UR5_2.smart_stop)
    T.start()

    # Obtain prerequisite data
    ENV_VALUES = env_data()

    # Start execution
    UR5_2.controller()
