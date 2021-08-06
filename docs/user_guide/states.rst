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

How to create a new state
#########################

| You can create new states and integrate them to GRIP. Although most of the time it is advised to create and :ref:`integrate an external component <integrate_software>`, which will generate a state for you, you might want to create your own set of states. We are going to review the required process below.

.. warning::
  All the interactive functionalities such as dropdown menu and automatic refresh of possible values for slots are **not** supported for external states.

1. Decide where to store the state
----------------------------------

| TODO

2. Content of the and name of the file
--------------------------------------

| The content of any state you want to create should follow this template:

.. code-block:: python

  #!/usr/bin/env python

  import smach
  # Import other packages if required

  # The name of the state MUST be the CamelCase version of the filename! For instance for this state the filename should
  # be name_of_state
  class NameOfState(smach.State):

      """
        Small description of what the state does
      """
      # Change the argument_to_set by the name of the variable you want to be able to configure in the task editor
      # You can also set default values, such as argument_to_set2=10
      # The remaining arguments from outcomes onward MUST NOT BE REMOVED
      # You can add more outcomes if you want in he signature
      def __init__(self, argument_to_set1, argument_to_set2, outcomes=["success", "finished"], input_keys=[], output_keys=[], io_keys=[]):
        """
          @param outcomes: Possible outcomes of the state. Default "success" and "failure"
          @param input_keys: List enumerating all the inputs that a state needs to run
          @param output_keys: List enumerating all the outputs that a state provides
          @param io_keys: List enumerating all objects to be used as input and output data
        """
          # Initialise the state, THIS LINE MUST NOT BE REMOVED
          smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys, input_keys=input_keys, output_keys=output_keys)
          # Do whatever you want here
          # ...
          # Outcomes of the state. THIS LINE MUST NOT BE REMOVED
          self.outcomes = outcomes

      # Do NOT change the name or the signature of this function
      def execute(self, userdata):
          """
            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] depending on your implementation
                     - outcomes[0] otherwise
          """
          # The code here is executed everytime that the state is run
          # So write what you need
          # ...
          # But don't forget to at least return one outcome!
          # Can be self.outcomes[-1] or self.outcomes[2] if you have more than two outcomes. If you do have more than 2,
          # make sure to have them defined in outcomes in the class signature!!!
          return self.outcomes[0]

.. warning::
  | Regardless of where you are storing the file, make sure to name the file following the underscore naming rule!
  | For instance if your state is named :code:`ComputeJointState`, the name of the file must be :code:`compute_joint_state.py`.
