.. _state_select:
******
Select
******

| This state allows you to extract a message from one of the managers and save it in the :code:`userdata`, i.e. making it accessible by all the states. Most of the provided states include this options, but this state can help you if using third-party states.

Outcomes
########

| This state has two outcomes: :code:`success` (green socket) and :code:`failure` (red socket). If the configuration of the state is wrong, or that the ROS msg cannot be retrieved, the state will return the outcome :code:`failure`. Otherwise return :code:`success` and stores the retrived message in the :code:`userdata`.

How to configure
################

| Once dropped in the task editor, you should see three parameters appear in the state:
* **input**: Name under which the ROS msg to retrieve has been saved inside the manager.
* **input_type**: Type of the manager from which the ROS msg should be retrieved. Please only use the values available in the associated dropdown widget.
* **output**: Name you want to give to the extracted message. If left empty, **names the retrieved ROS msg with the value of the parameter** :code:`input`. It must be a string.

| The ROS msg will be stored in the :code:`userdata` of the state machine. This means that other states will be able to access its value, if referenced by name.

Example of use
##############

You can find a concrete example of how to use this state, :ref:`here <example_state_select>`.
