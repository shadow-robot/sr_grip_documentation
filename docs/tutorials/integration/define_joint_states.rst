.. _define_js:
**********************************
Defining pre-recorded joint states
**********************************

This page contains a step-by-step guide to define pre-recorded joint states that you can use in the task editor. This is useful to define *waypoints* (e.g. recurrent joint states the robots will visit).

Prerequisites
#############
In this tutorial, we assume that a robot is already properly interfaced to GRIP (see :ref:`here <tutorials_list>` for the different integration modalities).

Procedure
#########
1. In the :code:`Settings` tab, go to the :code:`Named joint states` editor and create a new file by clicking on :code:`New`
2. In the margin, click on the :code:`+` symbol and enter the name of the joint state (i.e. waypoint) you want to create
3. Add as many joint states you want in this editor. Note that each one must respect the following format:

.. code-block:: yaml

    waypoint1_name:
      joint_name_1: <value_11>
      joint_name_2: <value_21>
      ...
      joint_name_n: <value_n1>

.. image:: ../img/joint_states.png

.. note::

	Make sure each created joint state has an **unique name** (i.e two joint states should have two distinct names). The joint states defined here can also be used to :ref:`create trajectories <define_traj>`.
