.. _make_simulation_compatible:
******************************
Running your robot with Gazebo
******************************

| GRIP features a simulation mode that allows you to quickly make sure that what you want to deploy works properly. We are going to explain in this page how to make sure you can use it with your robot. GRIP relies on `Gazebo <http://gazebosim.org/>`_, which is natively installed with ROS.
| **Disclaimer: The purpose of the simulation mode is mainly to make sure all components work as expected, not to provide a high performance simulation environment to develop new methods!**

Before starting
###############
| For now, the simulated controllers to actuate the joints of your robot must compatible with the `ROS control <http://wiki.ros.org/ros_control>`_ framework. So if you want to integrate your own controller, please follow this `tutorial <http://gazebosim.org/tutorials/?tut=ros_control>`_.
| Please note that you **must** have a MoveIt! configuration package for your robot to use GRIP's simulation mode. If you don't have one, you can find more details about how to create one :ref:`here <creating_moveit_config>`.
