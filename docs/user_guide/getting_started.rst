***************
Getting started
***************

This page introduces the different notions and steps required to start working with GRIP.

Getting familiar with the environment
#####################################

| As specified in the :ref:`instructions to install GRIP <install-instructions>`, the framework is deployed using `Docker <https://docs.docker.com/get-started/overview/>`_.
| If it's the first time you are hearing about docker and you want to know more about it, we advise you to read this `simplified explanation <https://medium.com/free-code-camp/docker-simplified-96639a35ff36#06d9>`_. In a nutshell, you can work in the Terminator window that pops up when you start the container without worrying about corrupting your native Ubuntu session (and that's pretty nice!).
| The container that you have installed is running Ubuntu 16.04, i.e. all the software that you can find natively on a fresh install of Ubuntu are there as well. And good news, the full ROS Kinetic stack is already installed for you! So you don't need to install MoveIt! or any other software that is installed with ROS.
| Note that instead of using the default **Gazebo 7.x**, GRIP natively runs **Gazebo 9.x**.

How to navigate inside the container
************************************
| When the Terminator window opens, you will be located at :code:`/home/user/`. From this point, you can use all the classic bash commands to navigate within the file system (e.g. :code:`cd`, :code:`ls`, :code:`mv`, etc.).
| To move directly to the catkin workspace, you can run

.. prompt:: bash $

    cd projects/shadow_robot/base

or

.. prompt:: bash $

    roscd && cd ..

A catkin workspace is a folder where you modify, build and install ROS packages. It must contain three folders:

- **src**: must contain the different ROS packages to build and run
- **devel**: development space
- **build**: build space

No need to worry, everything has already been set up for you and is ready to be used without further knowledge. However, if you want to know more about catkin and workspaces, you can find more `here <http://wiki.ros.org/catkin/conceptual_overview>`_ and `here <http://wiki.ros.org/catkin/workspaces>`_.

What is already there
*********************
| If you are having a look at what is inside the **src** folder of the catkin workspace, don't be scared if you only see the folder :code:`grip`, you will be able to install a lot of other ROS packages along with it. In fact, we made this container as *plain* as possible to avoid any confusion and potential conflict.
| This gives you the freedom to install whatever you want inside your container. For instance, if you want to test a work that relies on specific simualtors such as `MuJoCo <http://www.mujoco.org/>`_, you can install them without any additional overhead.
| We haven't included any additional and fancy text editor or IDEs, so everyone can install its favorite one (e.g. `Visual Studio Code <https://code.visualstudio.com/>`_, `JetBrains IDEs <https://www.jetbrains.com/>`_, `atom <https://atom.io/>`_, etc.).

What is GRIP
############

| GRIP is a ROS-based robot-agnostic software that allows for visual programming and fast prototyping of robotic grasping and manipulation tasks. The main purpose of this tool is to facilitate the integration of software and hardware components into complex robotic systems.
| Although the Robot Operating System (ROS) is not the only middleware available to operate robots, it remains one of the most used, especially in academia. For this reason, instead of re-implementing numerous features already provided by this operating system, we decided to make the most of it.
| If you've never used ROS and are reluctant to thoroughly learn this stack, GRIP is made for you. In fact, we tried to reduce as much as possible the ROS knowledge required to use our framework, to make sure GRIP remains accessible and easy to use. In addition, we provide a documentation with several examples for all the steps from robot integration to task design and execution.
| In order to simplify all the steps necessary to run a robot task, we have implemented an intuitive and reactive Graphical User Interface (GUI) for both the integration stage and task design. If you want to have a look at the code, all related files are located in the `grip_api <https://github.com/shadow-robot/sr_grip/tree/kinetic-devel/grip_api>`_ package. The code corresponding to all the other components of this fraemwork, can be found in the `grip_core <https://github.com/shadow-robot/sr_grip/tree/kinetic-devel/grip_core>`_ package.
| **Disclaimer: GRIP's focus is not to be highly performant, but rather to provide users a tool to easily reproduce their own setups and integrate components to screen them, before potentially integrating them to their existing optimised pipeline!**

If everything is clear so far, you can move to the :ref:`tutorials <tutorials_list>`.
