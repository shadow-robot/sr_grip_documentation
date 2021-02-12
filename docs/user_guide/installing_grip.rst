.. _install-instructions:
*************************
Installing the framework
*************************

Our software is deployed using Docker. Docker is a container framework where each container image is a lightweight, stand-alone, executable package that includes everything needed to run it. It is similar to a virtual machine but with much less overhead. Follow the instructions detailed below to get the latest Docker container up and running.

Hardware specifications
#######################

In order to run our software and the ROS software stack you will need to meet some hardware requirements.

* CPU: Intel i5 or above
* RAM: 4GB or above Hard Drive: Fast HDD or SSD (Laptop HDD can be slow)
* Graphics Card: Nvidia GPU (optional)
* OS: Ubuntu 16.04 or 18.04 (Active development)

The most important one is to have a fast HDD or an SSD.

Downloading the container
#########################

We have created a one-liner that is able to install Docker, download the image and create a new container for you. To use it, you first need to have a PC running Ubuntu (16.04 or 18.04 tested).

Prerequisite
*************
We **strongly** advise to run the one-liner on a machine without any version of docker installed. If you have never installed it, you can skip to the next subsection. If you are already using Docker, please be aware that the resulting container *might* not work. If it is the case, you can run the following lines:

.. prompt:: bash $

   sudo apt purge -y docker-engine docker docker.io docker-ce
   sudo apt autoremove -y --purge docker-engine docker docker.io docker-ce

These instructions are uninstalling docker but should not remove any of the containers already stored on your machine.

If the machine you are using to run the framework has a Nvidia card **and the Nvidia drivers are on**, then execute the following line

.. prompt:: bash $

   bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e nvidia_docker=true tag=kinetic-v0.0.6 reinstall=true demo_icons=false desktop_icon=false ethercat_right_hand="" image=shadowrobot/grip_framework container_name=<container_name>

You can change `<container_name>` by the name you want to give to the container that you are going to use. For instance, if you want your container to be named *GRIP_test*, then you would need to run the following command:

.. prompt:: bash $

    bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e nvidia_docker=true tag=kinetic-v0.0.6 reinstall=true demo_icons=false desktop_icon=false ethercat_right_hand="" image=shadowrobot/grip_framework container_name=GRIP_test

.. note::
    If you don't have a Nvidia graphic card or are running on the Xorg drivers, please use **nvidia_docker=false**

Troubleshooting
***************
When running the one-liner, if you encounter an error saying:

**/dev/fd/63: line 246:  1728 Segmentation fault      (core dumped) pip3 install --user -r ansible/data/ansible/requirements.txt**

please run :code:`rm -rf ~/.local` (make sure to make a copy of important files if you have any) and re-run the one-liner.

Running the container
#####################

| **We are going to assume in this section that you named your container *GRIP_test* in the one-liner!**
|
| At this stage, you should have both a docker image and container created for you. First, let's make sure that's the case. You should see the following

.. code-block:: bash

    $ docker ps -a
    CONTAINER ID   IMAGE                                       ...   NAMES
    3447e1i08b16   shadowrobot/grip_framework:kinetic-devel    ...   GRIP_test

To start the container, you just need to run

.. prompt:: bash $

    docker start GRIP_test

| A new `Terminator <https://gnometerminator.blogspot.com/p/introduction.html>`_ window will pop up, and will allow you to navigate inside the container. **None** of the operations you are going to run in this terminal will affect your native Ubuntu session.
| For instance if you install a text editor in the container, you won't be able to run in in your graphic Ubuntu session! So feel free to install and configure your favorite text editor and everything that you need to work efficiently.

Future releases
###############

For now, the docker that you have downloaded contains Ubuntu 16.04 and ROS Kinetic. We are currently working on the release of the framework using Ubuntu 18.04 and ROS Melodic. Stay tuned!
