#! /usr/bin/env python
"""
Library of UR5-specific functions for MoveIt! and RViz Motion Planning.
Contains:

1) MoveIt! Parameters and Controllers for controlling the arm
  
  a) Go to a specified pose
  b) Go to specified join angles
  c) Play a saved trajectory from a file

2) RViz Planning Scene controls for adding, attaching, detaching and removing objects from the Planning Scene
"""
import sys
import yaml
import tf
import actionlib
import moveit_commander
import moveit_msgs.msg
import rospkg
import rospy
from std_srvs.srv import Empty


class Ur5Moveit(object):
    """
    Class for the UR5 Arm, containing essential methods for execution.

    :var str _planning_group: The name to be assigned to the move_group
    :var str _box_name: The name to be stored to the box in the planning scene
    :var _computed_plan: The plan generated by the planner that will be executed
    :var str _pkg_path: The file path for the package
    :var str file_path: Extends from _pkg_path to point towards the folder containing saved trajectories.
    """

    _planning_group = "manipulator"
    _box_name = ''
    _computed_plan = ''
    _pkg_path = rospkg.RosPack().get_path('pkg_task5')
    file_path = _pkg_path + '/config/saved_trajectories/'


    # Constructor
    def __init__(self, arg_robot_name):
        """
        Constructor containing all essential MoveIt and RViz assets.
        """
        # Initialise node
        rospy.init_node('node_moveit_%s' % arg_robot_name, anonymous=True)

        self.tf_listener = tf.TransformListener()

        self._robot_ns = '/' + arg_robot_name

        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description",
            ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(
            self._planning_group,
            robot_description=self._robot_ns + "/robot_description",
            ns=self._robot_ns)

        rospy.Publisher(
            self._robot_ns + '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        actionlib.SimpleActionClient(
            self._robot_ns + '/execute_trajectory',
            moveit_msgs.msg.ExecuteTrajectoryAction).wait_for_server()

        self._touch_links = self._robot.get_link_names(
            group=self._planning_group)

        # Current State of the Robot is needed to add box to planning scene
        self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._group.get_planning_frame()) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(
                self._group.get_end_effector_link()) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._robot.get_group_names()) + '\033[0m')

        rospy.loginfo("Package Path: {}".format(self.file_path))

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):
        """
        Goes to a specified pose and orientation.

        :arg arg_pose: The pose and orientation to execute planning towards.
        :type arg_pose: Pose object

        :return: Confirmation whether the planning and execution was successful or not.
        :rtype: bool 
        """

        # Get current pose values
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        # Set final pose target with given pose value
        self._group.set_pose_target(arg_pose)
        self._group.set_start_state_to_current_state()
        rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        # Get final joints values calculated
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        # Verify planning/executing success
        if flag_plan:
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def hard_go_to_pose(self, arg_pose, arg_max_attempts):
        """
        Uses :meth:`go_to_pose`, but repeatedly tries to find a solution.

        :arg arg_pose: The pose and orientation to execute planning towards.
        :type arg_pose: Pose object
        :arg int arg_max_attempts: The maximum number of attempts for the planner.
        """
        number_attempts = 0
        flag_success = False

        while (number_attempts <= arg_max_attempts) and (flag_success is False):
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # # self.clear_octomap()

        return True

    def clear_octomap(self):
        """
        Clears the octomap of plans
        
        :return: A call to the :meth:clear_octomap_service_proxy` function
        """
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):
        """
        Goes to specified joint angles.

        :arg arg_list_joint_angles: A list of joint angles in radians to plan towards.
        :type arg_list_joint_angles: float[]

        :return: Confirmation whether the planning and execution was successful or not.
        :rtype: bool
        """

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.set_start_state_to_current_state()
        rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        """
        Uses :meth:`set_joint_angles`, but repeatedly tries to find a solution.

        :arg arg_list_joint_angles: List of joint angles to plan towards.
        :type arg_list_joint_angles: float[]
        :arg int arg_max_attempts: The maximum number of attempts for the planner.
        """

        number_attempts = 0
        flag_success = False

        while (number_attempts <= arg_max_attempts) and (flag_success is False):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """
        Plays a saved trajectory from a file.

        :arg str arg_file_path: The file path of the .yaml file with the saved trajectory.
        :arg str arg_file_name: The name of the .yaml file.

        :return: Confirmation of the execution.
        """
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open, Loader=yaml.Loader)

        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    def moveit_hard_play_planned_path_from_file(
            self, arg_file_path, arg_file_name, arg_max_attempts):
        """
        Uses :meth:`moveit_play_planned_path_from_file` but tries repeatedly for a solution.

        :arg str arg_file_path: The file path of the .yaml file with the saved trajectory.
        :arg str arg_file_name: The name of the .yaml file.
        :arg int arg_max_attempts: The maximum number of attempts for the planner.
        """
        number_attempts = 0
        flag_success = False

        while (number_attempts <= arg_max_attempts) and (flag_success is False):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # # self.clear_octomap()

        return True

    # Adds box to world
    def add_box(self, box_name, box_length, box_pose):
        """
        Adds a box to the RViz planning scene.

        :arg str box_name: The name to be assigned to the box.
        :arg float box_length: The size of the box.
        :arg box_pose: The pose and orientation of the box.
        :type box_pose: PoseStamped object
        """
        self._scene.add_box(box_name,
                            box_pose,
                            size=(box_length, box_length, box_length))

    def attach_box(self, box_name):
        """
        Attaches the specified object(box) in the planning scene to the robot hand.

        :arg str box_name: The name of the box in the RViz Planning Scene te be attached.
        """
        self._scene.attach_box(self._group.get_end_effector_link(),
                               box_name,
                               touch_links=self._touch_links)

    def detach_box(self, box_name):
        """
        Detaches the specified object(box) from the robot hand in the planning scene.

        :arg str box_name: The name of the box in the RViz Planning Scene to be detached.
        """
        self._scene.remove_attached_object(self._group.get_end_effector_link(),
                                           name=box_name)

    def remove_box(self, box_name):
        """
        Removes the specified object(box) from the RViz Planning Scene.

        :arg str box_name: The name of the box to be removed.
        """
        self._scene.remove_world_object(box_name)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
