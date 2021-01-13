.. _creating_moveit_config:
*********************************
Creating a MoveIt! config package
*********************************

| `MoveIt! <https://moveit.ros.org/documentation/concepts/>`_ is a ROS-compatible framework for grasping and manipulation tasks, which is commonly used in the research community. This software embeds kinematics libraries, controllers, and motion planners for robotic hands/arms. Due to an active and large community, a wide range of MoveIt! configurations are available for robot arms and hands. So, before going any further, **make sure that you cannot find one online**, on github for instance.
| If that's not the case, MoveIt! proposes a very useful tool, the `setup assistant <http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html>`_ to help you doing so. The only thing that you need is an `URDF file <https://industrial-training-master.readthedocs.io/en/kinetic/_source/session3/Intro-to-URDF.html>`_ file of your robot (generally provided by the robot's manufacturer).
| However, if you don't manage to properly create one or are lost because you don't understand what the assistant is asking you (no worries, it happens), we provide here a detailed step-by-step tutorial about how to manually create one.

.. sectnum::

Copying an existing moveit_config package
#########################################
Instead of creating a moveit config package from scratch, we are going to start from an existing one. First create an empty folder, which name will be the name of the moveit config package (we advise you to make it end with *_moveit_config*) and copy the content of this [repository](https://github.com/ARQ-CRISP/template_moveit_config.git) inside it. You can also initialize a repository if you want to keep it in git. Your folder should now contain two folders and two files, as follows:

.. code-block:: bash
    +-- config
    |
    +-- launch
    |
    CMakeLists.txt
    package.xml
