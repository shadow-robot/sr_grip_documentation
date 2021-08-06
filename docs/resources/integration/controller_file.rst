.. _ros_controller_file:
*****************************
What is a ROS controller file
*****************************
| A *ROS controller file* includes all the information ROS requires to operate robots. In GRIP, such files are required when you integrate a robot with MoveIt!, or if you have implemented your own controller following this `tutorial <http://wiki.ros.org/ros_control/Tutorials/Creating%20a%20controller>`_.

.. note::
    If your robot is composed of both an arm and a robot hand (and integrated through MoveIt!), you must specify one ROS controller file for the arm and one for the hand.

Each controller file must follow this template:
.. code-block:: yaml

    <controller_name>:
      type: "<controller_type>"
      joints:
        - <joint_name_1>
        - ...
        - <joint_name_n>
      constraints:
        goal_time: <goal_time>
        stopped_velocity_tolerance: <velocity_tolerance>
        <joint_name_1>: {trajectory: <value>, goal: <value>}
        <joint_name_2>: {trajectory: <value>, goal: <value>}
        ...
        <joint_name_m>: {trajectory: <value>, goal: <value>}
      stop_trajectory_duration: <trajectory_duration>
      state_publish_rate: <publish_rate>
      action_monitor_rate: <monitor_rate>
      allow_partial_joints_goal: <boolean>

| If you create your own controller (i.e., derived from :code:`controller_interface::ControllerBase`, see `here <http://wiki.ros.org/ros_control/Tutorials/Creating%20a%20controller>`_) declared in a plugin, you must define it in this file as well. Please note that some of the options, such as *constraints*, *stop_trajectory_duration* or others may not be relevant, i.e. depends on the controller type and/or your implementation.
| When using MoveIt!, please make sure that the controllers defined in such files **match the same name** as the MoveIt! controllers defined in :code:`controllers.yaml` of your moveit configuration package. If you are using your own ROS controller, you **don't** need to include it in the moveit config package.
