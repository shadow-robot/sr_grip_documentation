.. _hw_interface:
********************************
What is a ROS hardware interface
********************************

This page provides **non-exhaustive** explanations about what is a ROS hardware interface, and how to create one.

Overview
########

The role of the hardware interface is to provide an interface between ROS controllers and any physical robot. It is part of the `ROS control <http://wiki.ros.org/ros_control>`_ framework that makes sure a robot can be operated via controllers managed by ROS. Its role is better explained on this figure (extracted from the *ros_control* documentation)

.. image:: https://github.com/ros-controls/ros_control/blob/noetic-devel/ros_control/documentation/gazebo_ros_control.png

How to create one
#################

| Before starting implementing anything, you might want to make sure you don't already have one provided. For this, you can search for :code:`hardware_interface:RobotHW` in all relevant packages. If you can find one, you should not need to start from scratch then!
| If you don't have any and you want to create one, please see this `tutorial <https://github.com/ros-controls/ros_control/wiki/hardware_interface>`_, or this `one <https://medium.com/@slaterobotics/how-to-implement-ros-control-on-a-custom-robot-748b52751f2e>`_.

.. note::

	Having a hardware interface for your robot would allow you to integrate your own controllers inside the ROS control framework and then make MoveIt! run them

Compatibility with GRIP
***********************
| If you already have a ROS hardware interface for your robot 9or part of your robot), you might need to slightly change it. Don't worry, it should not take you too much time!
| For GRIP to be able to read and properly communicate with your robot, the given hardware interface must have a **parameter-less** constructor and must have an :code:`init` method with the following signature: :code:`init(ros::NodeHandle &n, ros::NodeHandle &robot_hw_nh)`. You can find example of working hardware interfaces for the `Franka panda arm <https://github.com/ARQ-CRISP/franka_ros/blob/bdenoun/multi_modified_for_gazebo/franka_combinable_hw/src/franka_combinable_hw.cpp>`_ and `Universal Robot arms <https://github.com/shadow-robot/sr_ur_arm/blob/kinetic-devel/sr_ur_robot_hw/src/sr_ur_robot_hw.cpp>`_ (the latter is already included in GRIP).

.. _example_connection_files:
Hardware connection file
########################

| Now that you have your hardware interface implemented, if you want GRIP to operate your real robot with MoveIt!, you will need to specify a hardware connection file. This YAML file must contain all the required parameters that enables the ROS control framework to operate your robot. These parameters are **implementation-dependent**.
| These files **must** respect the following format:

.. code-block:: yaml

    robot_hardware:
      - <hw_interface_name1>
      # If you have two robot arms or two robot manipulators
      - <hw_interface_name2>

    <hw_interface_name1>:
      type: <hw_interface_type>
      # Then comes all the hardware interface implementation-dependent parameters.

For instance, with `this <https://github.com/shadow-robot/sr_ur_arm/blob/kinetic-devel/sr_ur_robot_hw/src/sr_ur_robot_hw.cpp>`_ hardware interface implementation (for Universal Robot arms), the hardware connection file should look like this:

.. code-block:: yaml

    robot_hardware:
      - ur_robot_hw

    ur_robot_hw:
      type: sr_ur_robot_hw/UrArmRobotHW
      # Usually it is the prefix you give to your robot in the URDF file
      robot_id: ""
      # Robot's IP address (change this to your robot's)
      robot_ip_address: 186.58.01.6
      # Your laptop/desktop's IP address (change this to yours)
      control_pc_ip_address: 186.58.01.9
      # Robot arm speed scale [0.0, 1.0] (works natively only with Universal Robots)
      speed_scale: 0.5
      # Extra end-effector weight (initial payload of the robot, e.g. gripper weight)
      payload_mass_kg: 0
      # Manipulator payload centre of inertia (estimated)
      payload_center_of_mass_m: [0, 0, 0]
      # Topic published when the controllers for the hardware must be loaded
      topic_to_wait_for: arm_ready
      # Path to the directory in which the robot programs are stored (only for Universal Robots).
      robot_program_path: /home/user/projects/shadow_robot/base_deps/src/sr_ur_arm/sr_ur_bringup/robot_programs/

If you are using `this <https://github.com/ARQ-CRISP/franka_ros/blob/bdenoun/multi_modified_for_gazebo/franka_combinable_hw/src/franka_combinable_hw.cpp>`_ hardware interface for the Franka panda arm, this file should look like this:

.. code-block:: yaml

    robot_hardware:
      - franka_robot_hw

    franka_robot_hw:
      type: franka_combinable_hw/FrankaCombinableHW

      # Usually it is the prefix you give to your robot in the URDF file
      robot_id: panda

      # Robot's IP address (change this to your robot's)
      robot_ip: 15.11.0.2

      # Joint names
      joint_names:
        - panda_joint1
        - panda_joint2
        - panda_joint3
        - panda_joint4
        - panda_joint5
        - panda_joint6
        - panda_joint7
