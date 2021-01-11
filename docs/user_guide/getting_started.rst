***************
Getting started
***************

This page introduces the different notions and steps required to start working with GRIP.

Getting familiar with the environment
#####################################

As specified in the :ref:`instructions to install GRIP <install-instructions>`, the framework is deployed using `Docker <https://docs.docker.com/get-started/overview/>`_.
If it's the first time you are hearing about docker and you want to know more about it, we advise you to read this `simplified explanation <https://medium.com/free-code-camp/docker-simplified-96639a35ff36#06d9>`_.
In a nutshell, you can work in the Terminator window that pops up when you start the container without worrying about corrupting your native Ubuntu session (and that's pretty nice!).
| The container that you have installed is running Ubuntu 16.04, i.e. all the software that you can find natively on a fresh install of Ubuntu are there as well. And good news, the full ROS Kinetic stack is already installed for you! So you don't need to install MoveIt! or any other software that is installed with ROS.
| Note that instead of using the default **Gazebo 7.x**, GRIP natively runs **Gazebo 9.x**.

How to navigate inside the container
************************************
When the Terminator window opens, you will be located at :code:`/home/user/`. From this point, you can use all the classic bash commands to navigate within the file system (e.g. :code:`cd`, :code:`ls`, :code:`mv`, etc.).
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
If you are having a look at what is inside the **src** folder of the catkin workspace, don't be scared if you only see the folder :code:`grip`, you will be able to install a lot of other ROS packages along with it.
In fact, we made this container as *plain* as possible to avoid any confusion and potential conflict.
| This gives you the freedom to install whatever you want inside your container. For instance, if you want to test a work that relies on specific simualtors such as `MuJoCo <http://www.mujoco.org/>`_, you can install them without any additional overhead.
| We haven't included any additional and fancy text editor or IDEs, so everyone can install its favorite one (e.g. `Visual Studio Code <http://www.mujoco.org/>`_, `JetBrains IDEs <https://www.jetbrains.com/>`_, `atom <https://atom.io/>`_, etc.).

What is GRIP
############
To be continued
