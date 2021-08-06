.. _state_allow_collisions:
***************
AllowCollisions
***************

| This state allows you to allow/disallow collisions between some part of the robot and its environment. Allowing and disallowing the collisions rely on MoveIt!, so only groups that are operated via MoveIt! will work.
| You can for instance, allow the collision between the fingers of your robot hand, or allow the collision between an object of the scene and the gripper.

Outcomes
########

| This state has two outcomes: :code:`success` (green socket) and :code:`failure` (red socket). If the configuration of the state is wrong, or that modification of the allowed collisions has failed, :code:`failure` is returned. Return the other outcome if the operation is successful.

How to configure
################

| Once dropped in the task editor, you should see two or three parameters appear in the state:
* **group_name**: Name of the MoveIt! group for which the collision checking should be changed. This slot will not appear if only one group is defined.
* **allow**: Boolean stating whether the collisions should be enabled or disabled. Please only use the values available in the associated dropdown widget.
* **objects**: Optional list of objects for which you want the the collision to be modified. If left empty all added objects will be considered

Example of use
##############

You can find a concrete example of how to use this state, :ref:`here <example_state_allow_collisions>`.
