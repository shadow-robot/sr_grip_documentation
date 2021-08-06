.. _states_list:
******
States
******

| This page contains all you need to know about states in GRIP, i.e. how to create new states and how to use the provided ones.

Constant states
###############

| The source code of the following states can be found `here <https://github.com/shadow-robot/sr_grip/tree/kinetic-devel/grip_core/src/grip_core/states>`_. You should always be able to see them, regardless of the configuration of your robot.

:ref:`Counter <state_counter>`
******************************

:ref:`ReinitialiseManagers <state_reinitialise_managers>`
*********************************************************

:ref:`Select <state_select>`
****************************

:ref:`WaitFor <state_wait_for>`
*******************************

Commander states
################

| The source code of the following states can be found `here <https://github.com/shadow-robot/sr_grip/tree/kinetic-devel/grip_core/src/grip_core/states/commander>`_. These states will become available if and only if your robot uses MoveIt to operate a part of your robot.

.. note::
  | For these states to appear at least **one** MoveIt motion planner must be defined in the robot integration tab.
  | If **more than one** planner is defined, a configuration slot named :code:`group_name` will appear for all these states. It allows you to choose which part of the robot you wish to configure.

:ref:`AllowCollisions <state_allow_collisions>`
***********************************************

:ref:`ComputePlan <state_compute_plan>`
***************************************

:ref:`ExecuteTrajectory <state_execute_trajectory>`
***************************************************

:ref:`Move <state_move>`
************************

Generated states
################

| When you are interfacing an :ref:`external component <integrate_software>` or a :ref:`sensor <integrate_sensor>`, GRIP will automatically generate the code of the corresponding states.

:ref:`ExternalComponent <state_external_component>`
***************************************************

:ref:`Sensor <state_sensor>`
****************************
