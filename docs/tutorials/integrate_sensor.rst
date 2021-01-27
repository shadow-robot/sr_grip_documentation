.. _integrate_sensor:
********************
Integrating a sensor
********************

This page contains a step-by-step guide to integrate a sensor to GRIP.

What you need before starting
#############################

* A ROS node or launch file that runs the sensor and publish the collected data on topics.
* The list of the topics

Prerequisites
#############
Examples will be provided using a Kinect v2. If you want to replicate this tutorial, make sure to install `libfreenect2 <https://github.com/OpenKinect/libfreenect2/blob/master/README.md#linux>`_ and to follow these `instructions <https://github.com/code-iai/iai_kinect2#install>`_ to install iai_kinect2 in :code:`/home/user/projects/shadow_robot/base/src/`

Procedure
#########

1. Start the framework: :code:`roslaunch grip_api start_framework.launch`
2. Specify the URDF file of the robot to the framework
3. Set the composition of your robot(s), i.e. how many arms, hands and sensors need to be configured. In this case we are going to set only one sensor, but you can have several sensors
4. In the :code:`Settings` tab, in the :code:`Sensors config`, click on :code:`New` and save the file wherever you want
5. In the margin, click on the :code:`+` symbol and enter the name of your sensor
6. A template will appear in the editor with the following fields:
    * :code:`data_topics`: Dictionary that must contain all the different data to collect with their associated topic
    * :code:`frame_id`: Name of the frame associated to the sensor in the scene
    * :code:`initial_pose`: Pose of the sensor in the scene

.. image:: ../img/sensor_integration.png

7. Click on :code:`Save` in the editor and here you are, your sensor is now integrated (i.e. you can see a state in the :code:`Task editor` tab that allows you to collect data from the referenced topics).
8. Integrate the rest of your robot
9. **Before** clicking on :code:`Launch robot`, make sure you run the node or launch file publishing the data on topics in another terminal. In our case, we would first run :code:`roslaunch kinect2_bridge kinect2_bridge.launch reg_method:=cpu depth_method:=opengl` before clicking on :code:`Launch robot`

.. note::

	Instead of defining the pose in the editor, you can directly refer to poses defined in the :ref:`pose editor <define_poses>`. For instance if a pose named :code:`sensor_pose` is defined in the pose editor, you can set :code:`initial_pose: sensor_pose`.
