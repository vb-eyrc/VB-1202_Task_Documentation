#! /usr/bin/env python
"""
This script controls the UR5 near the shelf for placing boxes on the conveyor.
It borrows methods from :mod:`lib_task5` for execution, and
uses multithreading to operate :meth:`controller <UR5One.controller>` and the order pipeline
simultaneously.
"""
import os
import json
import datetime
import threading
import rospy, time
from rospy.exceptions import ROSInterruptException

from pkg_ros_iot_bridge.msg import msgMqttSub  # ROS-IoT Bridge MQTT
from std_msgs.msg import String
from lib_task5 import Ur5Moveit  # The library for the UR5s
from node_iot_action_client import RosIotBridgeActionClient  # ROS-IoT Bridge Client


class UR5One(object):
    """
    Class for the module.

    :var list(dict) order_list: A queue of total received orders to dispatch.
    :var list(dict) hp_list: Queue of High-Priority packages.
    :var list(dict) mp_list: Queue of Medium-Priority packages.
    :var list(dict) lp_list: Queue of Low Priority packages.
    :var list(str) package_colours: Stores the packages colours from :meth:`package_callback`
    :var rospy.Publisher pub_dispatched_order: Publisher for sending dispatch order stats to UR5_2.
    :var rospy.Subscriber sub_package_colour: Subscriber for :meth:`package_callback`
    """

    def __init__(self):
        """
        Constructor containing essential data.
        """
        self.order_list = []  # A queue of total orders received to be dispatched
        self.hp_list = []  # Queue of High-Priority packages
        self.mp_list = []  # Queue of Medium-Priority packages
        self.lp_list = []  # Queue of Low Priority packages
        self.package_colours = []  # String list to store packages colours from package_callback()

        self.pub_dispatched_order = rospy.Publisher('/dispatched_order', String, queue_size=10)

        rospy.Subscriber('/ros_iot_bridge/mqtt/sub',
                         msgMqttSub,
                         self.order_callback)

        # Subscriber for Shelf Camera for decoding package colours
        # This is only required to get data once

        self.sub_package_colour = \
            rospy.Subscriber('/package_colour',
                             String,
                             self.package_callback)

    def order_callback(self, msg_order):
        """
        Callback function for incoming orders Subscriber.
        Generates a queue of orders to be dispatched.

        :arg msgMqttSub.msg msg_order: The online order data sent by the server to the client.
        """

        order = json.loads(msg_order.message)
        # The order received, separated into its separate fields as a python dictionary

        # Setting priority and price based on item description
        if order["item"] == "Medicine":
            priority = "HP"
            cost = "450"
        elif order["item"] == "Food":
            priority = "MP"
            cost = "250"
        elif order["item"] == "Clothes":
            priority = "LP"
            cost = "150"

        if order:
            # Filling in order details
            order = {"id": "IncomingOrders",
                     "Team Id": "VB#1202",
                     "Unique Id": "isAmiTvb",
                     "Order ID": order["order_id"],
                     "Order Date and Time": order["order_time"],
                     "Item": order["item"],
                     "Priority": priority,
                     "Order Quantity": order["qty"],
                     "City": order["city"],
                     "Latitude": order["lat"],
                     "Longitude": order["lon"],
                     "Cost": cost}

            # Updating Spreadsheet
            ACTION_CLIENT.send_goal_pls(order)
            self.order_list.append(order)  # Updating the queue with the current order
            self.priority_sorter()  # Sorting the orders by priority


    def package_callback(self, msg_package_list):
        """
        Callback function for :attr:`sub_package_colours <UR5One>`.
        Formats the colour list received from the shelf camera
        into an operable list.

        :arg list(str) msg_package_list: CSV-formatted list of all the package colours in order.
        """

        temp_var = msg_package_list.data.split(',')  # Converting the CSV

        if len(temp_var) == 9 and 'NA' not in temp_var:
            rospy.logwarn("Configuration stored:")
            rospy.logwarn(temp_var)
            # Once data has been retrieved once it is not required anymore
            self.sub_package_colour.unregister()
            self.package_colours = temp_var


    def priority_sorter(self):
        """
        Sorts the order queue according to priority, into 
        :attr:`hp_list <UR5One>`, :attr:`mp_list <UR5One>`,
        and :attr:`lp_list <UR5One>`.
        """
        self.hp_list = []
        self.mp_list = []
        self.lp_list = []

        if len(self.order_list) >= 1:
            i = 0
            # Iterating through the Queue to sort the orders by priority
            while i < len(self.order_list):
                curr_order = self.order_list[i]
                if curr_order["Priority"] == "HP":
                    self.hp_list.append(curr_order)
                elif curr_order["Priority"] == "MP":
                    self.mp_list.append(curr_order)
                elif curr_order["Priority"] == "LP":
                    self.lp_list.append(curr_order)
                i += 1


    def shelf_plan(self, order):
        """
        .. autometh: shelf_plan

        Conducts pick-place planning for the packages on the shelf.

        :arg dict order: The current order to dispatch.
        """

        package_list = ['R0CO', 'R0C1', 'R0C2', 'R1C0',
                        'R1C1', 'R1C2', 'R2C0', 'R2C1', 'R2C2']  # Package numbers on the shelf
        # List of packages that use a different place configuration
        place_config_list = [1, 3, 4, 6, 8, 9]
        packagen = 0  # Will be used to pick the selected prioritised package

        # Selecting the package to pick based on priority
        if order["Priority"] == "HP":
            packagen = self.package_colours.index("red")
            self.package_colours[packagen] = ''
        elif order["Priority"] == "MP":
            packagen = self.package_colours.index("yellow")
            self.package_colours[packagen] = ''
        elif order["Priority"] == "LP":
            packagen = self.package_colours.index("green")
            self.package_colours[packagen] = ''

        # Log operation
        rospy.loginfo(
            "\033[96m" +
            "Executing dispatch for package {} with order ID {}".format(package_list[packagen],
                                                                        order["Order ID"]) +
            "\033[0m")

        # Execute pick operation
        UR5.moveit_hard_play_planned_path_from_file(
            UR5.file_path + 'ur5_1/', 'home_to_pkg{}.yaml'.format(packagen + 1), 1)
        # Activate gripper
        os.system\
(
    'rosservice call /eyrc/vb/ur5/activate_vacuum_gripper/ur5_1 "activate_vacuum_gripper: true"'
)

        # Execute place operation
        if packagen + 1 in place_config_list:
            config = 'l'  # Optimised place based on package position
        else:
            config = 'r'
        UR5.moveit_hard_play_planned_path_from_file(
            UR5.file_path + 'ur5_1/',
            'pkg{}_to_place_{}.yaml'.format(packagen + 1, config), 1)
        # Deactivate gripper
        os.system\
(
    'rosservice call /eyrc/vb/ur5/activate_vacuum_gripper/ur5_1 "activate_vacuum_gripper: false"'
)

        # Update dispatch status
        rospy.sleep(0.1)
        self.pub_dispatched_order.publish(json.dumps(order))
        order["Dispatch Date and Time"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Updating Spreadsheet
        ACTION_CLIENT.send_goal_pls(order)

        # Log operation
        rospy.loginfo(
            "\033[96m" + "Package for order ID {} dispatched!".format(order["Order ID"])
            + "\033[0m")

        # Travel back to home for the next operation
        UR5.moveit_hard_play_planned_path_from_file(
            UR5.file_path + 'ur5_1/', 'place_{}_to_home.yaml'.format(config), 1)

        return order


    def controller(self):
        """
        Main execution. Finalises the Order Dispatch from :meth:`priority_sorter` and controls the UR5_1.
        """
        rospy.sleep(15)

        # Travelling to home position to prepare for operation
        rospy.loginfo(
            '\033[96m' + "Travelling to home" + '\033[0m')
        UR5.moveit_hard_play_planned_path_from_file(UR5.file_path +
                                                    'ur5_1/', 'zero_to_home.yaml', 1)
        curr_order = []

        while not rospy.is_shutdown():
            try:
                if self.hp_list:
                    dispatch_list = self.hp_list
                    curr_order = dispatch_list[0]
                    del self.hp_list[0]
                    self.order_list.remove(curr_order)
                elif self.mp_list:
                    dispatch_list = self.mp_list
                    curr_order = dispatch_list[0]
                    del self.mp_list[0]
                    self.order_list.remove(curr_order)
                elif self.lp_list:
                    dispatch_list = self.lp_list
                    curr_order = dispatch_list[0]
                    del self.lp_list[0]
                    self.order_list.remove(curr_order)

                if curr_order:
                    # Updating dispatch information
                    order = {"id": "OrdersDispatched",
                             "Team Id": "VB#1202",
                             "Unique Id": "isAmiTvb",
                             "Order ID": curr_order["Order ID"],
                             "City": curr_order["City"],
                             "Item": curr_order["Item"],
                             "Priority": curr_order["Priority"],
                             "Dispatch Quantity": curr_order["Order Quantity"],
                             "Cost": curr_order["Cost"],
                             "Dispatch Status": "YES",
                             "Dispatch Date and Time": ""}

                    # Controlling the UR5_1
                    self.shelf_plan(order)
            except ROSInterruptException:
                pass


if __name__ == '__main__':
    UR5 = Ur5Moveit("ur5_1")  # Initialising the UR5

    # Start the ROS-IoT Bridge
    ACTION_CLIENT = RosIotBridgeActionClient()

    # Creating Object of UR5One
    UR5_1 = UR5One()

    T = threading.Thread(target=UR5_1.controller)
    T.start()

    rospy.spin()
