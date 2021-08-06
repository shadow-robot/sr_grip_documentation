.. _state_execute_trajectory:
*****************
ExecuteTrajectory
*****************

| This state allows you to execute an already computed joint trajectory. This can be used to simplify the execution of predetermined and constant motions going through known waypoints.

Outcomes
########

| This state has two outcomes: :code:`success` (green socket) and :code:`failure` (red socket). If the configuration of the state is wrong, or if the `JointTrajectory <http://docs.ros.org/en/indigo/api/trajectory_msgs/html/msg/JointTrajectory.html>`_ cannot be executed by MoveIt!, :code:`failure` is returned. Return the other outcome if the operation is successful.

How to configure
################

| Once dropped in the task editor, you should see one or two parameters appear in the state:
* **group_name**: Name of the MoveIt! group that should execute the given trajectory. This slot will not appear if only one group is defined.
* **trajectory_name**: Name of the trajectory to execute. By default, will look for it inside the :code:`userdata`. If not found, will be retrieved from the corresponding manager.

Example of use
##############

You can find a concrete example of how to use this state, :ref:`here <example_state_execute_trajectory>`.
