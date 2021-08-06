.. _creating_description_files:
*************************************************
What are description files and how to create them
*************************************************

This page explains what are description files, why we need them and how to create them. We can distinguish Gazebo :code:`.world` files that describe the simulated environment, the associated models that are simulated and ROS :code:`.scene` files that contain information about the collisions.

.. note::
    For convenience sake, you can gather all these files in a folder (or even better a ROS package), like this `one <https://github.com/ARQ-CRISP/ARQ_common_packages/tree/master/arq_description_common>`_.

Gazebo models folder
####################
This folder must contain at least all the objects used to compose a world, defined as `SDF files <http://gazebosim.org/tutorials?tut=build_model>`_.
The minimal directory structure of such a folder is the following:
.. code-block:: bash
    models
    +-- <model_using_mesh_name>
    |   +-- meshes
    |       +-- <model_using_mesh_name>.stl
    |   +-- model.config
    |   +-- model.sdf
    +-- <model_no_mesh_name>
    |   +-- model.config
    |   +-- model.sdf

Please note that the meshes can also be *.dae* files, allowing you to add texture. You can find an example of a gazebo models folder `here <https://github.com/ARQ-CRISP/ARQ_common_packages/tree/master/arq_description_common/models>`_.

Gazebo world files
##################
Gazebo worls allow you to simulate your robot's environement. It can be either static or contain some dynamics mechanisms. It exists several ways of creating gazebo worlds. We detail here three options.

Using the Gazebo interface
**************************
If you want to use the native Gazebo interface, you can follow this `tutorial <http://gazebosim.org/tutorials?tut=build_world>`_.

Using the Shadow World Generator
********************************
We have implemented a software that helps in the process of creating Gazebo world files. All the dependencies are already installed in the container. You just need to follow these `instructions <https://shadow-experimental.readthedocs.io/en/latest/user_guide/1_6_software_description.html#creating-a-new-world-scene>`_.

Manually creating world files
*****************************
| If you are not a big fan of GUIs and/or you already have a good hang of how to design Gazebo world files, you can still quickly create your world file manually.
| The **most important** part is to have all the objects that will be used to create your world in **a single** Gazebo models folder. In this case, you can just modify the following template and save the file with a *.world* extension, and that's it you have your Gazebo world created!

.. code-block:: xml
    <?xml version="1.0" ?>
    <!-- Must be a sdf file -->
    <sdf version="1.5">
        <world name="default">
            <!-- Add a source of light, which is very important if embedding a camera -->
            <light name='sun' type='directional'>
                <cast_shadows>1</cast_shadows>
                <pose frame=''>0 0 10 0 -0 0</pose>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <specular>0.1 0.1 0.1 1</specular>
                <attenuation>
                    <range>1000</range>
                    <constant>0.9</constant>
                    <linear>0.01</linear>
                    <quadratic>0.001</quadratic>
                </attenuation>
                <direction>-0.5 0.5 -1</direction>
            </light>
            <!-- Include one after the other the different objects you want to add -->
            <include>
                <!-- the "model://" refers to the Gazebo models path and "my_first_model" refers to the name of the folder containing, the sdf and config file-->
                <uri>model://my_first_model</uri>
                <!-- Whether you want to make the object static (meaning that it will ignore all applied forces) -->
                <static>true</static>
                <!-- Name you want to give to the model -->
                <name>my_first_model</name>
                <!-- Create the reference frame that is going to be used by the pose provided in the model file -->
                <pose>0.0 0.0 0.0 0 0 0</pose>
            </include>
            <!-- model2 -->
            <include>
                <uri>model://my_second_model</uri>
                <static>true</static>
                <name>my_second_model</name>
                <pose>0.0 0.0 0.0 0 0 0</pose>
            </include>
            <!-- Physics parameters -->
            <physics type="ode">
                <gravity>0.000000 0.000000 -9.810000</gravity>
                <ode>
                    <solver>
                        <type>quick</type>
                        <iters>100</iters>
                        <precon_iters>0</precon_iters>
                        <sor>1.000000</sor>
                    </solver>
                    <constraints>
                        <cfm>0.000000</cfm>
                        <erp>0.500000</erp>
                        <contact_max_correcting_vel>1000.000000</contact_max_correcting_vel>
                        <contact_surface_layer>0.00000</contact_surface_layer>
                    </constraints>
                </ode>
                <real_time_update_rate>0.000000</real_time_update_rate>
                <max_step_size>0.001000</max_step_size>
            </physics>
            <scene>
                <ambient>0.4 0.4 0.4 1</ambient>
                <background>0.7 0.7 1 1</background>
                <shadows>1</shadows>
            </scene>
            <!-- Display configuration -->
            <gui fullscreen='0'>
                <camera name='user_camera'>
                    <pose frame=''>2.8784 4.2586 1.43117 0 0.083643 -2.30699</pose>
                    <view_controller>orbit</view_controller>
                </camera>
            </gui>
        </world>
    </sdf>

.. note::
    This template is a backbone allowing to create simple (but most of the time sufficient) world files. You can freely adjust the different parameters to better fit your constraints.

If you want to add a sensor with its corresponding model, you can do so in this template. For instance adding a RGB-D sensor such as a `kinect <https://github.com/ARQ-CRISP/ARQ_common_packages/tree/master/arq_description_common/models/kinect_ros>`_, which will already create the ROS topics containing the data.

Scene files
###########
| Scene files contain all the information about the collision scene of the robot. You can either create a simplified version of your environment using primitive shapes (not recommended) or use the *.world* file that you have just created. You can either follow these `instructions <https://shadow-experimental.readthedocs.io/en/latest/user_guide/1_6_software_description.html#creating-a-new-world-scene>`_ or directly use GRIP to create the scene file.
| In simulation mode, specify the world file corresponding to the scene you want to generate. Once the robot is running in simulation, go to RViz. If you don't have the :code:`Motion Planning` on the left part of your window, add it. Then go in the tab :code:`Scene Objects` and click on :code:`Export As Text`. Choose the path to the folder in which you want to save the scene, pick a name and save it. And here is your scene file.
