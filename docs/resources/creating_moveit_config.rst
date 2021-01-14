.. _creating_moveit_config:
****************************************
Creating a MoveIt! configuration package
****************************************

| `MoveIt! <https://moveit.ros.org/documentation/concepts/>`_ is a ROS-compatible framework for grasping and manipulation tasks, which is commonly used in the research community. This software embeds kinematics libraries, controllers, and motion planners for robotic hands/arms. Due to an active and large community, a wide range of MoveIt! configurations are available for robot arms and hands. So, before going any further, **make sure that you cannot find one online**, on github for instance.
| If that's not the case, MoveIt! proposes a very useful tool, the `setup assistant <http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html>`_ to help you doing so. The only thing that you need is an `URDF file <https://industrial-training-master.readthedocs.io/en/kinetic/_source/session3/Intro-to-URDF.html>`_ file of your robot (generally provided by the robot's manufacturer).
| However, if you don't manage to properly create one or are lost because you don't understand what the assistant is asking you (no worries, it happens), we provide here a detailed step-by-step tutorial about how to manually create one.

1. Copy an existing moveit_config package
############################################
Instead of creating a moveit config package from scratch, we are going to start from an existing one. First create an empty folder in :code:`/home/projects/shadow/base/src`, which name will be the name of the moveit config package (we advise you to make it end with *_moveit_config*) and copy the content of this `repository <https://github.com/ARQ-CRISP/template_moveit_config.git>`_ inside it. You can also initialize a repository if you want to keep it in git. Your folder should now contain two folders and two files, as follows:

.. code-block:: bash

    +-- config
    |
    +-- launch
    |
    CMakeLists.txt
    package.xml

2. Change the package description
###################################
| Replace all the information (i.e. the description, the maintainer, the author and the name) of the :code:`package.xml` file. In line 2 of :code:`CMakeLists.txt`, modify what's inside :code:`project()`. The name inside :code:`project()` in the :code:`CMakeLists.txt` and the name inside :code:`<name>` tage of :code:`package.xml` **must correspond to the name of your folder!**. We encourage you to just replace :code:`template_moveit_config` by the name of your folder in those two files.
| To make sure that everything is working fine, please run the following:

.. prompt:: bash $

    cd /home/projects/shadow_robot/base
    catkin_make

If it does not finish successfully, it might be that the name of the fodler and the name you changed in :code:`package.xml` and :code:`CMakeLists.txt` don't match!

3. Fit the content of the package to your robot
###############################################
| Now, you can search all the occurrences of :code:`template_moveit_config` and replace them with your folder's name. Once this is done (it is **very important** to do the previous step first), you can search for all the occurrences of :code:`template` and replace them with the name you want to give to your robot (let's assume you name it *my_robot*).

.. warning::
    Make sure to change the filenames as well

| Next, if your robot is composed of a hand or gripper, replace all the :code:`<manipulator_name>` with the name you want to give to your manipulator. Similarly, find all the occurrences of :code:`<manipulator_finger_i>` and to change them to the name you want to give for each finger.

4. Changing the urdf and srdf files
###################################
| First, change :code:`my_robot.urdf.xacro` (located in the folder :code:`config`) to the **urdf.xacro** file that describes both your arm and manipulator (if you have both). The content of :code:`my_robot.urdf.xacro` should help you do that. Otherwise, you can have a look `here <https://github.com/ARQ-CRISP/arq_ur5_ezgripper_moveit_config/blob/master/config/arq_ur5_with_ezgripper.urdf.xacro>`_ for an example. Please note that you **don't** need to clone the :code:`ur_description` package in your catkin workspace, since GRIP supports natively Universal Robot arms. Once this is done, change :code:`<base_link_of_the_robot>` to the name of the base link of the robot arm.
| Second, you need to change the content of the `SRDF <http://wiki.ros.org/srdf>`_ file. In :code:`my_robot.srdf`, change the description of each link between :code:`<>` with their real names. For instance of Universal Robot (UR) arms, :code:`<ee_link_of_robot_arm>` becomes :code:`ee_link`. Make sure to do it everywhere in the file and to add the proper disabled collisions. For this part, we strongly advise to use the moveit setup assistant (and then copy-paste the result). You can find an example of SRDF file can be found `here <https://github.com/ARQ-CRISP/arq_ur5_ezgripper_moveit_config/blob/master/config/arq_ur5_with_ezgripper.srdf>`_.

.. note::
    For each element surrounded by :code:`<>`, it is easier to search for it in the whole folder so you avoid any incompatibility.

5. Specify the name of your robot's joints
##########################################
The last step consists in specifying the name of the joints for each group we want MoveIt! to be able to control. Replace the proper values in :code:`controllers.yaml`, :code:`fake_controllers.yaml` and :code:`joint_limits.yaml`.

6. Test that everything works
#############################
The first sanity check is to run
.. prompt:: bash $

    cd /home/projects/shadow/base
    catkin_make

If it compiled without any error, you can try to follow :ref:`this tutorial <integrate_with_moveit>` to see if your configuration package works fine. You should be able to move your robot in `Rviz <http://wiki.ros.org/rviz>`_ (and Gazebo if the simulation mode is on).

Troubleshooting
###############
If for some reasons it does not work, you can try to have a look at `this <https://github.com/ARQ-CRISP/arq_ur5_ezgripper_moveit_config>`_, or `this <https://github.com/ARQ-CRISP/panda_moveit_config>`_ moveit config package and see what differs.
