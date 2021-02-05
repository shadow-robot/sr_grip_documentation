.. _automated_pick_place:
****************************************
Running an automated pick and place task
****************************************

| This page contains a step-by-step guide to design and execute automated pick and place in GRIP's task editor.
| This tutorial copes with automated (and not autonomous) tasks, therefore the robot will only move between poses or joint states already registered in GRIP's interface.

What you need before starting
#############################

A robot arm and gripper or hand integrated to GRIP, either :ref:`through MoveIt! <integrate_with_moveit>`, via a :ref:`launch file <integrate_with_launch>` or using :ref:`external software <integrate_software>` to control the hardware.

Prerequisites
#############

In order to illustrate the same example both when the robot is integrated through MoveIt! and when one part of the robot is controlled by an external controller, we are going to use a UR5 robot arm with a EZGripper. If you want to replicate this tutorial, please clone `this repository <https://github.com/ARQ-CRISP/arq_ur5_ezgripper_moveit_config.git>`_ and `this one <https://github.com/ARQ-CRISP/EZGripper>`_ in :code:`/home/user/projects/shadow_robot/base/src`. Then compile them

.. prompt:: bash $

    cd /home/user/projects/shadow_robot/base
    catkin_make
    source devel/setup.bash

Pick and place with full MoveIt! integration
############################################
In this section, we are going to run our robot in simulation **because we cannot control the physical EZGripper via MoveIt!**. Note that if you can operate your physical robot through MoveIt!, you should read and reproduce similar steps!

1. Integrate your robot following :ref:`this tutorial <integrate_with_moveit>`. In our case, we have declared two planners, one for the group :code:`arm_and_manipulator` and the other one for :code:`ezgripper_finger_1` ()
