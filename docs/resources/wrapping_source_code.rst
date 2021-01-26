.. _wrap_code:
*****************************************************
Making ROS action/service server compatible with GRIP
*****************************************************

This page provides **non-exhaustive** explanations about what are ROS service and action servers and how to create them.

ROS services
############

| In addition to ROS `topics <http://wiki.ros.org/Topics>`_, that provide a one-to-many way to communicate between different ROS `nodes <http://wiki.ros.org/Nodes>`_ (i.e. process performing computations), ROS also supports *request / reply* communications, through services. A ROS service is defined using `srv <http://wiki.ros.org/srv>`_ files that describe the type of `ROS message <http://wiki.ros.org/Messages>`_ is expected as request and as reply. The computations performed by the service are gathered in a server (i.e. ROS node), and a client calls this service by sending the request message and awaiting the reply.
| If you don't understand the details, no worries, you don't have to master these notions to successfully create your service! In fact, you can just follow our step-by-step :ref:`guide <ros_services>` to create ROS services **fully compatible** with GRIP!

ROS actions
###########
| Similarly to services, actions provide a *request / reply* type of communication between a server and a client. However, in some cases, the operations performed by the server can take a long time to execute. In such scenarios, it can be interesting to cancel the request during execution or to get periodic feedback about the current progress. That's what `actions <http://wiki.ros.org/actionlib>`_ do.
| If you are not sure about how to create ROS actions, you can just follow our step-by-step :ref:`guide <ros_actions>` to create ROS actions **fully compatible** with GRIP!

Compatibility with GRIP
#######################
| If you want your ROS action and/or service to be properly integrated to GRIP, then your **srv** and/or **action** files must follow a specific format.

| For ROS servers, your **srv** files must follow this format:

.. code-block:: bash

    <msg_type> input
    ---
    int8 outcome
    <msg_type> returned_object

| For ROS action, your **action** files must follow this format:

.. code-block:: bash

    # Goal definition
    <msg_type> input
    ---
    # Result definition
    <msg_type> returned_object
    int8 outcome
    ---
    # Feedback definition
    <msg_type> <field_name>

| :code:`<msg_type>` is left to your preference, i.e. it's implementation-dependent. As a result, when specified to GRIP's GUI, the framework will automatically create a state running your code in the task editor!

.. note::

	If you have followed this :ref:`resource <ros_services>` or this :ref:`one <ros_actions>` to create your servers, you don't need to do anything else!
