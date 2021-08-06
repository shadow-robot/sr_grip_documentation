.. _state_compute_plan:
***********
ComputePlan
***********

| This state allows you to compute a plan between two states of the robot. This is a crucial component in making a robot move from a point A to a point B.

Outcomes
########

| This state has two outcomes: :code:`success` (green socket) and :code:`failure` (red socket). If the configuration of the state is wrong, or if a motion plan cannot be computed for the designated group, :code:`failure` is returned. Return the other outcome if the operation is successful.

How to configure
################

| Once dropped in the task editor, you should see five or six parameters appear in the state:
* **group_name**: Name of the MoveIt! group for which the plan will be computed. This slot will not appear if only one group is defined.
* **target_type**: Name of the manager from which the target state should be retrieved. Valid values are :code:`joint state` and :code:`pose`. **If left empty**, the target will be **retrieved from the userdata**.
* **target_name**: Name of the target state. It can be the name of either a pose or a joint state which is located either in a manager or in the userdata.
* **plan_name**: Name to give to the plan. **If left empty** stores the resulting plan in the commander as the last computed one. If not left empty, sends the computed plan to its manager.
* **starting_type**: Name of the manager from which the starting state should be retrieved. Valid values are :code:`joint state` and :code:`pose`. **If left empty**, the starting state will be **retrieved from the userdata**.
* **starting_name**: Name of the starting state. It can be the name of either a pose or a joint state which is located either in a manager or in the userdata. If **this slot and starting_type** are left **empty**, the current state of the robot (when executing :code:`ComputePlan`) will be used as starting state.

Example of use
##############

You can find a concrete example of how to use this state, :ref:`here <example_state_compute_plan>`.
