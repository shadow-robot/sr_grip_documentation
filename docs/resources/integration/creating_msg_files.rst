.. _create_msg:
*************************
Creating custom msg files
*************************

This page provides **non-exhaustive** explanations about what are ROS messages and how to create them.

Description
###########
| The different ROS nodes (i.e processes performing computation) communicate with each other using messages. They are sent (or published) via `topics <http://wiki.ros.org/Topics>`_. `msg <http://wiki.ros.org/msg>`_ files are files that specify the data structure of the message (i.e. specifying the type of the fields).
| You can find more detailed explanations about ROS messages `here <http://wiki.ros.org/Messages>`_.

Creating your own message
#########################
| ROS natively provides a `wide range of msgs <http://wiki.ros.org/common_msgs>`_ that you can use with GRIP. However, you might want to create msgs files that better fit the output of a script instead of changing your implementation to fit a specific msg type.
| In this :ref:`resource <wrap_code>`, we provide an example of a node that randomly generates a position within provided boundaries. These values were fed as a list, and we had to specifically check that there are indeed 6 values sent to the node. In the following, we are going to create a cutom msg file that ensures we have 6 values.

1. Creating a msg folder
************************
First, go in your ROS repository (in our case, we are using this `one <https://github.com/ARQ-CRISP/server_template>`_), and if you don't already have one, create a :code:`msg` folder.

2. Create your msg file
***********************
| In your :code:`msg` folder, create a new file ending with *.msg*. In our case, we are going to call it :code:`Boundaries.msg`.
| The content of your msg file should respect the following format:

.. code-block:: bash

    <type_field_1> <name_field_1>
    <type_field_2> <name_field_1>
    <type_field_3> <name_field_1>

If our case, the content of :code:`Boundaries.msg` will be

.. code-block:: bash

    float32 x_min
    float32 x_max
    float32 y_min
    float32 y_max
    float32 z_min
    float32 z_max

.. note::

	The type of the fields can be other ROS messages, as shown `here <https://github.com/shadow-robot/sr_grip/blob/kinetic-devel/grip_core/msg/RobotPose.msg>`_.

3. Modify the :code:`package.xml`
*********************************
In the :code:`package.xml` file of your repository, you need to have the following:

.. code-block:: xml

    <build_depend>message_generation</build_depend>
    <exec_depend>message_runtime</exec_depend>

Note that if you are using the `server_template <https://github.com/ARQ-CRISP/server_template.git>`_ package, you don't need to change anything.

4. Modify the :code:`CMakeLists.txt`
************************************
| First, search for :code:`find_package` in your file and make sure to have :code:`message_generation` among the other packages listes after :code:`catkin REQUIRED COMPONENTS`.
| Second, search for :code:`catkin_package`, and on the same line as :code:`CATKIN_DEPENDS` add :code:`message_runtime` (potentially among other packages).
| Third, make sure the :code:`add_message_files` block in not commented, and add on the line after :code:`FILES` the name of your msg file. In your case, we would have this:

.. code-block:: CMake

    add_message_files(
      FILES
      Boundaries.msg
    )

| Last, make sure the :code:`generate_messages` block is not commented and add on a new line after :code:`DEPENDENCIES` the messages or msg file depends on. In our case, we use float32 that comes from `std_msgs <http://wiki.ros.org/std_msgs>`_:

.. code-block:: CMake

    generate_messages(
      DEPENDENCIES
      std_msgs
    )

.. note::

	If you are using the `server_template <https://github.com/ARQ-CRISP/server_template.git>`_ package, the two first steps are done for you and the :code:`CMakeLists.txt` already include type of msgs as dependencies.

5. Recompile your package
*************************
The last step to be able to use your msg files is to recompile your packages. To do so, run

.. prompt:: bash $

    cd /home/user/projects/shadow_robot/base
    catkin_make
    source devel/setup.bash

And you are done and have successfully created your message!

Using your message
##################
Once you are done with the previous instructions (be sure to have run the last step!), you can use your messages in Python and C++ scripts within your ROS package. The following subsections provide minimal code to send and receive messages in Python and C++ (based on the previous generated msg).

Python
******
Minimal code to create and send a message through a topic

.. code-block:: python

    #!/usr/bin/env python

    import rospy
    # Import the defined msg file
    from server_template.msg import Boundaries


    if __name__ == '__main__':
        # Register the node to ROS
        rospy.init_node('send_boundaries')

        # Create a publisher that will send Boundaries msgs to all the nodes that will listen to the created topic
        # First argument is the name of the created topic, second is the type of the msg and the
        # last one is the maximum number of msgs kept before removing them
        boundaries_publisher = rospy.Publisher("topic_boundaries", Boundaries, queue_size=1)

        # Create and fill the message
        message_to_send = Boundaries()
        # Values here are arbitary
        message_to_send.x_min = -1.5
        message_to_send.x_max = 2.3
        message_to_send.y_min = 1.05
        message_to_send.y_max = 3.5
        message_to_send.z_min = 6.02
        message_to_send.z_max = 8.95

        # Create a loop that will continuously send the same message through the topic
        while not rospy.is_shutdown():
            boundaries_publisher.publish(message_to_send)

Minimal code to receive and process messages (here the processing is just to disaply its content)

.. code-block:: python

    #!/usr/bin/env python

    import rospy
    # Import the defined msg file
    from server_template.msg import Boundaries


    def print_msg(received_msg):
        """
            Callback that will be triggered upon reception of each message. Must have an argument.
        """
        rospy.loginfo("Content of the received message:")
        rospy.loginfo("x min: {}".format(received_msg.x_min))
        rospy.loginfo("x max: {}".format(received_msg.x_max))
        rospy.loginfo("y min: {}".format(received_msg.y_min))
        rospy.loginfo("y max: {}".format(received_msg.y_max))
        rospy.loginfo("z min: {}".format(received_msg.z_min))
        rospy.loginfo("z max: {}".format(received_msg.z_max))
        print("")

    if __name__ == '__main__':
        # Register the node to ROS
        rospy.init_node('receive_boundaries')

        # Create a subscriber that will receive Boundaries msgs that are published on the given topic
        # First argument is the name of the topic to lsiten to, second is the type of the msg and the
        # last one is the function that must be called upon reception of a message
        boundaries_receiver = rospy.Subscriber("topic_boundaries", Boundaries, print_msg)

        # Keep listening even after first msg reception
        rospy.spin()

C++
***
Minimal code to create and send a message through a topic

.. code-block:: cpp

    #include <ros/ros.h>
    // Import the defined msg file
    #include <server_template/Boundaries.h>


    int main(int argc, char **argv)
    {
      // Register the node to ROS
      ros::init(argc, argv, "send_boundaries");
      // Create a node handler
      ros::NodeHandle node_handle;
      /**
      Create a publisher that will send Boundaries msgs to all the nodes that will listen to the created topic
      First argument is the name of the created topic, second is the maximum number of msgs kept before removing them
      */
      ros::Publisher boundaries_publisher = node_handle.advertise<server_template::Boundaries>("topic_boundaries", 1);

      // Create and fill the message
      server_template::Boundaries message_to_send;
      // Values here are arbitary
      message_to_send.x_min = -1.5;
      message_to_send.x_max = 2.3;
      message_to_send.y_min = 1.05;
      message_to_send.y_max = 3.5;
      message_to_send.z_min = 6.02;
      message_to_send.z_max = 8.95;

      // Create a loop that will continuously send the same message through the topic
      while (ros::ok())
      {
          boundaries_publisher.publish(message_to_send);
          ros::spinOnce();
      }

      return 0;
    }

Minimal code to receive and process messages (here the processing is just to disaply its content)

.. code-block:: cpp

    #include <ros/ros.h>
    // Import the defined msg file
    #include <server_template/Boundaries.h>

    // Callback that will be triggered upon reception of each message. Must have an argument.
    void print_msg(const server_template::Boundaries::ConstPtr& received_msg)
    {
        ROS_INFO("Content of the received message:");
        ROS_INFO_STREAM("x min: " << received_msg->x_min);
        ROS_INFO_STREAM("x max: " << received_msg->x_max);
        ROS_INFO_STREAM("y min: " << received_msg->y_min);
        ROS_INFO_STREAM("y max: " << received_msg->y_max);
        ROS_INFO_STREAM("z min: " << received_msg->z_min);
        ROS_INFO_STREAM("z max: " << received_msg->z_max);
        std::cout << "" << std::endl;
    }


    int main(int argc, char **argv)
    {
      // Register the node to ROS
      ros::init(argc, argv, "receive_boundaries");
      // Create a node handler
      ros::NodeHandle node_handle;
      /**
      Create a subscriber that will receive Boundaries msgs that are published on the given topic
      First argument is the name of the topic to listen to, second is the size of the message queue and
      last one is the function that must be called upon reception of a message
      */
      ros::Subscriber boundaries_publisher = node_handle.subscribe("topic_boundaries", 1, print_msg);

      // Keep listening even after first msg reception
      ros::spin();

      return 0;
    }
