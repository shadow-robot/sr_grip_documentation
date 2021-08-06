.. _state_sensor:
************
Sensor state
************

| When you :ref:`integrate a sensor <integrate_sensor>` to GRIP, a dedicated state will be generated for you. When this state is executed, it collects the data set on one of the topics you have declared when integrating it. The data will then be stored in the :code:`userdata`.

Outcomes
########

| This state has two outcomes: :code:`success` (green socket) and :code:`failure` (red socket). If no ROS msg can be captured from the specified topic after 2 seconds then return :code:`failure`. If the message is successfully stored in the userdata, return :code:`success`.

How to configure
################

| Once dropped in the task editor, you should see two parameters appear in the state:
* **sensor_topic**: Name of the topic, the ORS msg should be extracted from. It must be the name of one of the topics you have defined when integrating it. Please use the values avaialble in dropdown menu.
* **output**: Name under which the ROS msg will be saved in the :code:`userdata`. Must not be left empty.

Example of use
##############

You can find a concrete example of how to use this state, :ref:`here <autonomous_pick_place>`.
