.. _state_reinitialise_managers:
********************
ReinitialiseManagers
********************

| This state allows you to reset the managers, i.e. remove all the previously stored ROS msgs while keeping the one you have defined during the integration stage. It is especially useful when you plan to run the same task that involve the managers several times in a row (e.g. benchmark).

Outcomes
########

| This state has two outcomes: :code:`success` (green socket) and :code:`failure` (red socket). If the reinitialisation of **any** manager failed, the outcome :code:`failure` will be triggered. Otherwise return the first outcome.

How to configure
################

| This state is not configurable. So you just need to properly connect its sockets to the others states.

Example of use
##############

You can find a concrete example of how to use this state, :ref:`here <example_state_reinitialise_managers>`.
