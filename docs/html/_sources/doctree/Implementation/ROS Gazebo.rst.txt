==============
ROS and Gazebo
==============
.. sectnum::
   :start: 2

ROS
***
.. figure:: Images/ROS\ Terminal.png
   :align: center

   The ROS nodes for the arms in action inside the terminal.

Robot Operating System (ROS) is a middleware that interconnects many different sensors, robots, etc. as nodes in a control network, and helps maintain communication pipelines and control between the nodes, thus providing a robust modeling and control system for robotics applications. The task implementation utilized ROS Melodic Morenia, released in 2018, due to its stability and long-term support since it has been around for a long time.

Through encapsulated modular packages, each containing one or multiple nodes that are run through Python scripts that are collectively executed through launch files in each package, nodes can access configuration files containing formats to send signals to other nodes under, or preloaded data to access during execution, etc. The entire hierarchy of the ROS system implemented in this task can be summarised thus:

* :ref:`IoT-pkg`
  
  * :ref:`IoT-node`

* :ref:`Task5`

  * :ref:`Task5-Lib`

    * :ref:`Task5-UR51`
    * :ref:`Task5-UR52`

  * :ref:`Task5-package`
  * :ref:`Task5-client`


.. _IoT-pkg:

pkg_ros_iot_bridge
-------------------

.. _IoT-node:

node_ros_iot_bridge
+++++++++++++++++++

.. _Task5:

pkg_task5
---------

.. _Task5-Lib:

lib_task5
+++++++++

.. _Task5-UR51:

node_t5_ur5_1
"""""""""""""

.. _Task5-UR52:

node_t5_ur5_2
"""""""""""""

.. _Task5-package:

node_package_detect
+++++++++++++++++++

.. _Task5-client:

node_action_client
++++++++++++++++++

Gazebo
******

.. figure:: Images/GazeboSim.png
   :align: center

   The Gazebo environment provided for the task.

The following stuff was used in Gazebo
