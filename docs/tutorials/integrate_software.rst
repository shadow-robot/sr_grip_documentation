.. _integrate_software:
********************************
Integrating an external software
********************************

This page contains a step-by-step guide to integrate an external software component to GRIP.

What is an external software
############################
| As detailed :ref:`here <integrate_with_moveit>`, you can specify the controller, motion planner and kinematics to use to operate your robot through MoveIt!. However, you might want to use a controller, motion planner or kinematics library that you have previously developed and that is not integrated to MoveIt! or to the `ROS control <http://wiki.ros.org/ros_control>`_ framework. That's what we call *external* software, as it is not integrated to any ROS stack.
| For this reason, we have developed an interface to integrate such *external* components through GRIP's GUI that allows you to use them directly in the task editor.

.. note::
    This is the **same** procedure for **controllers**, **kinematics libraries**, **motion planners** (hardware configuration tabs) and for **high-level methods** (settings tabs).

What you need before starting
#############################

* The components to integrate wrapped inside ROS service/action servers (see :ref:`here <wrap_code>`)
* `URDF file <http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file>`_ of the robot
**Optional:** A MoveIt! configuration package (if you don't have one, see :ref:`here <creating_moveit_config>`)


**Documentation in progress**
