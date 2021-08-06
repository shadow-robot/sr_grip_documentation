.. _state_move:
****
Move
****

| This state allows you to execute an already computed motion plan. As long as the provided plan is of the same type as a MoveIt! plan (i.e. `RobotTrajectory <http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html>`_), this state can execute them so that the robot moves.

Outcomes
########

| This state has two outcomes: :code:`success` (green socket) and :code:`failure` (red socket). If the configuration of the state is wrong, or if the `JointTrajectory <http://docs.ros.org/en/indigo/api/trajectory_msgs/html/msg/JointTrajectory.html>`_ cannot be executed by MoveIt!, :code:`failure` is returned. Return the other outcome if the operation is successful.

How to configure
################

| Once dropped in the task editor, you should see one or two parameters appear in the state:
* **group_name**: Name of the MoveIt! group that should execute the given plan. This slot will not appear if only one group is defined.
* **plan_name**: Name of the trajectory to execute. By default, will look for it inside the :code:`userdata`. If not found, will be retrieved from the corresponding manager. **If left empty, execute the last plan computed by MoveIt! for the given group**.

Example of use
##############

You can find a concrete example of how to use this state, :ref:`here <example_state_move>`.
