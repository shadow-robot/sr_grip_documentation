.. _integrate_with_moveit:
********************************
Integrating a robot with MoveIt!
********************************

This page contains a step-by-step guide to integrate a robot using MoveIt!

What you need before starting
#############################

* A MoveIt! configuration package (if you don't have one, see :ref:`here <creating_moveit_config>`)
* `URDF file <http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file>`_ of the robot
* :ref:`ROS controllers <ros_controller_file>`

Prerequisites
#############
Examples will be provided using the Franka Panda arm and gripper. If you want to replicate this tutorial, please clone this `repository <https://github.com/ARQ-CRISP/panda_moveit_config>`_, this `repository <https://github.com/ARQ-CRISP/panda_moveit_config>`_ and this `one <https://github.com/ARQ-CRISP/ARQ_common_packages>`_ in :code:`/home/user/projects/shadow_robot/base/src`. Then compile them

.. prompt:: bash $

    cd ..
    sudo apt install ros-kinetic-libfranka
    catkin_make
    source devel/setup.bash

Procedure
#########

Common steps
************
1. Start the framework: :code:`roslaunch grip_api start_framework.launch`
2. Specify the URDF file file of the robot to the framework
3. Set the composition of your robot(s), i.e. how many arms, hands and sensors need to be configured
4. Set the path to the MoveIt! configuration package in the corresponding field. You should now see two editors allowing you to provide further configuration options (only if you need to).

.. image:: ../img/franka_moveit_step1.png

5. In each hardware config tab (arm and hand), set the corresponding ROS controllers you want to use. In our case, we use this `file <>`_ for the arm controller and this `one <>`_ for the gripper controller.
6. Create (or load) a new motion planner config file for the robot you want to integrate. Once the file is initialized in the editor, click on the + to add a new motion planner config. Follow the different dialog and add the following information:
    * test
