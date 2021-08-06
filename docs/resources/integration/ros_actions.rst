.. _ros_actions:
********************
Creating ROS actions
********************

| This page provides a step-by-step guide to create a ROS action from already existing code. Note that this tutorial includes some specific steps that ensures your action can be used by GRIP!
|
| Let's consider that you have a script that determines which cartesian point from a pre-determined list is the closest to the current robot's end effector position. To do so, let's assume that you have the following python code:

.. code-block:: python

    #!/usr/bin/env python

    from geometry_msgs.msg import Point
    import math

    # Predefined list of points
    WAYPOINTS = [[0., 0., 0.], [1.5, 0.0, -0.5], [0.5, 2.5, -0.8], [1.4, 2.6, 1.7], [-1.7, -1.5, 0.9]]


    def return_closest_point(ee_position):
        """
            Return a Point msg containing the closest point from the input Point msgs ee_effector
        """
        # Create a list containing the values of the position of the end effector set as input
        ee_pos = [ee_position.x, ee_position.y, ee_position.z]
        # At first we want min_dist to be a large number
        min_distance = 1000
        # Will store the point that is the closest to the end effector position
        min_point = list()
        for point in WAYPOINTS:
            distance = math.sqrt((ee_pos[0] - point[0])**2 + (ee_pos[1] - point[1])**2 + (ee_pos[2] - point[2])**2)
            if distance < min_distance:
                min_distance = distance
                # Copy the point inside the min_point variable
                min_point = point[:]

        output_msg = Point()
        output_msg.x = min_point[0]
        output_msg.y = min_point[1]
        output_msg.z = min_point[2]

        return output_msg

The C++ equivalent of this code would be the following:

.. code-block:: cpp

    #include <geometry_msgs/Point.h>
    #include <vector>

    std::vector<std::vector<double>> WAYPOINTS{ {0., 0., 0.}, {1.5, 0.0, -0.5}, {0.5, 2.5, -0.8}, {1.4, 2.6, 1.7},
                                               {-1.7, -1.5, 0.9} };

    geometry_msgs::Point return_closest_point(geometry_msgs::Point ee_position)
    {
       // Create a vector containing the values of the position of the end effector set as input
       std::vector<double> ee_pos{ ee_position.x, ee_position.y, ee_position.z};
       // At first we want min_dist to be a large number
       float min_distance = 1000.0;
       // Will store the point that is the closest to the end effector position
       std::vector<double> min_point;
       // Go over each cartesian point
       for (int index = 0; index < WAYPOINTS.size(); index++)
       {
           double distance = sqrt(pow(ee_pos[0] - WAYPOINTS[index][0], 2) + pow(ee_pos[1] - WAYPOINTS[index][1], 2) + pow(ee_pos[2] - WAYPOINTS[index][2], 2));
           if (distance < min_distance)
           {
               min_distance = distance;
               min_point = WAYPOINTS[index];
           }
       }

       // Now that the loop is done, create a Point msg
       geometry_msgs::Point output_msg;
       output_msg.x = min_point[0];
       output_msg.y = min_point[1];
       output_msg.z = min_point[2];

       return output_msg;
    }

The following instructions hold whether your code is implmented in C++ or Python!

1. Copy the server_template folder
##################################
Instead of creating a ROS package from scratch, we are going to start from an existing one. First create an empty folder in :code:`/home/projects/shadow/base/src`, which name will be the name of the ROS package containing your code and copy the content of this `repository <https://github.com/ARQ-CRISP/server_template.git>`_ inside it. You can also initialize a repository if you want to keep it in git. In our case, we named our ROS package :code:`example_package`. This folder contains five folders and seven files, as follows:

.. code-block:: bash

    +-- action
    |
    +-- include
    |   +-- server_template
    |       template_action_server.hpp
    |
    +-- msg
    |
    +-- scripts
    |   template_action_server.cpp
    |   template_action_server.py
    |   template_service_server.cpp
    |   template_service_server.py
    |
    +-- srv
    CMakeLists.txt
    package.xml

If your code is written in C++, all the headers (**.hpp** files) must be placed in :code:`/include/server_template`, while your **.cpp** files can be located in :code:`scripts`. If you are using python, just add your **.py** files in :code:`scripts`.

2. Change the package description
#################################
| Replace all the information (i.e. the description, the maintainer, the author and the name) of the :code:`package.xml` file. In line 2 of :code:`CMakeLists.txt`, modify what's inside :code:`project()`. The name inside :code:`project()` in the :code:`CMakeLists.txt` and the name inside :code:`<name>` tage of :code:`package.xml` **must correspond to the name of your folder!**. We encourage you to just replace :code:`server_template` by the name of your package in those two files.

.. warning::
    Make sure to change the name of the folder in :code:`include` as well!

| To make sure everything is working fine, please run the following:

.. prompt:: bash $

    cd /home/projects/shadow_robot/base
    catkin_make

If it does not finish successfully, it might be that the name of the folder and the name you changed in :code:`package.xml` and :code:`CMakeLists.txt` don't match!

3. Create msg and action files
##############################
| The **action** file will contain the backbone of what should be received and sent by the server. In order to be fully compatible with GRIP, the action file **must** respect the following format:

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

| :code:`<msg_type>` is left to your preference, i.e. it's implementation-dependent. You can use all the `built-in types <http://wiki.ros.org/msg>`_ or :ref:`create your own msg file <create_msg>`.
| In our case, the function we want to wrap inside a ROS action expects a :code:`Point` msg as input and outputs another :code:`Point` msg. For this reason, we are going to create a new file, :code:`GetClosestPoint.action`, in the :code:`action` folder with the following content

.. code-block:: bash

    # Goal definition
    geometry_msgs/Point input
    ---
    # Result definition
    geometry_msgs/Point returned_object
    int8 outcome
    ---
    # Feedback definition
    float64[] current_point
    float64 min_distance

| Before forgetting, let's edit the :code:`CMakeLists.txt` file. Go to line 72 and add the name of your action file after :code:`FILES` (if you have several, add one per line). In our case, we would get

.. code-block:: CMake

    add_action_files(
      FILES
      GetClosestPoint.action
    )

| To make sure everything works so far, run

.. prompt:: bash $

    cd /home/user/projects/shadow_robot/base
    catkin_make
    source devel/setup.bash

4. Filling the templates
########################
| If the code you want to wrap inside ROS services is written in C++, you will find the :code:`template_service_server.cpp` file in :code:`scripts`. If you are using python, the template file is :code:`template_service_server.py`. Although the syntaxes are different, the steps are exactly the same (and described in the files). You can either copy/paste and rename these files or just rename them. In our case we are going to create the file :code:`get_closest_point_server.py` (python version) and the files :code:`include/example_package/get_closest_point_server.hpp` and :code:`get_closest_point_server.cpp` (C++ version).

4.1 Import the generated action files
*************************************
Make sure to change the occurences of :code:`server_template` by your package name (e.g. :code:`example_package`). Then, change :code:`actionName` by the name of your action file without the extension (e.g. :code:`GetClosestPoint`).

4.2 Add your code
*****************
| Add your code in the file. It can either directly be a function written outside the class or a new method that you create inside the class. The most important is that your code must be run inside :code:`goal_callback()`.
| **In Python**, if you want to access the values stored in the :code:`input` field of your action file, you can use :code:`input_values = new_goal.input` and carry out any operation on the variable :code:`input_values` inside :code:`goal_callback`.
| Similarly, **in C++**, you can use :code:`<variable_type> input_values = new_goal_->input;` and then process as you wish the varaible :code:`input_values`.

4.3 Return the result
*********************
Make sure to fill and return the response of your server (in :code:`goal_callback()`), keeping in mind that it must contain the fields :code:`outcome` and :code:`returned_object`.

4.4 Call your code when the script is executed
**********************************************
Change the name of the ROS node that will run your server (string inside :code:`rospy.init_node()` or :code:`ros::init()`). The last step is to change the name of your action (i.e. argument at the last-but-one line).


In our example, the :code:`get_closest_point_server.py` looks like this:

.. code-block:: python

    #!/usr/bin/env python

    import rospy
    import actionlib
    # Change server_template by the name of your ROS package
    # and actionName by the name of your action file (without the .action)
    from example_package.msg import GetClosestPointAction, GetClosestPointResult, GetClosestPointFeedback

    # You can add here any other import statement you might need for your code
    from geometry_msgs.msg import Point
    import math

    # Predefined list of points
    WAYPOINTS = [[0., 0., 0.], [1.5, 0.0, -0.5], [0.5, 2.5, -0.8], [1.4, 2.6, 1.7], [-1.7, -1.5, 0.9]]

    # You can define here as many functions as you want


    # Class containing the action server (you can change the name, but don't forget to also change it line 72)
    class GetClosestPointServer(object):

        """
            Class running the action server to execute your code
        """

        def __init__(self, action_server_name):
            """
                Initialize the action server
            """
            # Change actionName by the name of your action file
            self.action_server = actionlib.SimpleActionServer(action_server_name, GetClosestPointAction, auto_start=False)
            # Set the callback to be executed when a goal is received
            self.action_server.register_goal_callback(self.goal_callback)
            # Set the callback that should be executed when a preempt request is received
            self.action_server.register_preempt_callback(self.preempt_callback)
            # Start the server
            self.action_server.start()

        def goal_callback(self):
            """
                Callback executed when a goal is received. Your code should be called or written inside this method
            """
            # The first step is to accept a new goal.
            new_goal = self.action_server.accept_new_goal()

            end_effector_position = new_goal.input

            # Create a list containing the values of the position of the end effector set as input
            ee_pos = [end_effector_position.x, end_effector_position.y, end_effector_position.z]
            # At first we want min_dist to be a large number
            min_distance = 1000
            # Will store the point that is the closest to the end effector position
            min_point = list()
            # For each point we can send a feedback
            for point in WAYPOINTS:
                distance = math.sqrt((ee_pos[0] - point[0])**2 + (ee_pos[1] - point[1])**2 + (ee_pos[2] - point[2])**2)
                if distance < min_distance:
                    min_distance = distance
                    # Copy the point inside the min_point variable
                    min_point = point[:]
                # Define the feedback
                action_feedback = GetClosestPointFeedback()
                # Set the current point currently tested
                action_feedback.current_point = point
                # Set the current minimum found
                action_feedback.min_distance = min_distance
                # Publish the feedback
                self.action_server.publish_feedback(action_feedback)

            # Now that the loop is done, create a Point msg
            output_msg = Point()
            output_msg.x = min_point[0]
            output_msg.y = min_point[1]
            output_msg.z = min_point[2]
            # Return the response of the action.
            self.action_result = GetClosestPointResult()
            # We only have one possible outcome here, so always send 0
            self.action_result.outcome = 0
            self.action_result.returned_object = output_msg
            self.action_server.set_succeeded(self.action_result)

        def preempt_callback(self):
            """
                Callback executed when a preempt request has been received.
            """
            rospy.loginfo("Action preempted")
            self.action_server.set_preempted()

    if __name__ == '__main__':
        # Initialise the node with a specific name (please change it to match your action)
        rospy.init_node('get_closest_python_action_server')
        # Create an instance of the class running the action server. The argument of the class determines the name of the
        # action. Make sure to change it to match your code.
        action_server = GetClosestPointServer("get_closest_point")
        rospy.spin()

Similarly, here is the content of :code:`get_closest_point_server.hpp`:

.. code-block:: cpp

    #ifndef GET_CLOSEST_POINT_ACTION_H
    #define GET_CLOSEST_POINT_ACTION_H

    #include <ros/ros.h>
    #include <string>
    #include <actionlib/server/simple_action_server.h>
    /**
    Please change server_template by the name of your ROS package
    and actionName by the name of your action file (without the action)
    */
    #include <example_package/GetClosestPointAction.h>
    #include <example_package/GetClosestPointFeedback.h>
    #include <example_package/GetClosestPointResult.h>
    #include <example_package/GetClosestPointGoal.h>

    // You can add here any other import statement you might need for your code
    #include <vector>
    #include <geometry_msgs/Point.h>
    #include <math.h>

    /**
    Class containing the action server (you can change the name, but don't forget to also change it everywhere)
     */
    class GetClosestPointServer
    {
      public:
        // Constructor
        GetClosestPointServer(ros::NodeHandle* node_handler, std::string action_server_name);

      private:
        // Change actionName by the name of your ROS package and action file
        actionlib::SimpleActionServer<example_package::GetClosestPointAction> action_server_;
        // Declare and initialise the message containing the outcome of the action
        example_package::GetClosestPointResult action_result_;
        // Declare and initialise a actionNameFeedback containing the feedback to send during the execution
        example_package::GetClosestPointFeedback action_feedback_;
        // Declare and initialise an actionNameGoal containing the goal sent to the server
        example_package::GetClosestPointGoalConstPtr new_goal_;
        // Declare and initialise a boolean storing the state of the server
        bool busy_ = false;

        // Internal method executing all the steps required when receiving a new goal or preempting an action
        void goal_callback();
        void preempt_callback();
    };

    #endif  // GET_CLOSEST_POINT_ACTION_H

And the content of :code:`get_closest_point_server.cpp`:

.. code-block:: cpp

    // Change server_name byt he name of your package and template_action_server by the name of your hpp file
    #include <example_package/get_closest_point_server.hpp>

    std::vector<std::vector<double>> WAYPOINTS{ {0., 0., 0.}, {1.5, 0.0, -0.5}, {0.5, 2.5, -0.8}, {1.4, 2.6, 1.7},
                                               {-1.7, -1.5, 0.9} };
    /**
     Constructor of the class
     * @param nodehandler               reference to a ros NodeHandle object
     * @param action_server_name        name given to the action server
     */
    GetClosestPointServer::GetClosestPointServer(ros::NodeHandle* node_handler, std::string action_server_name)
      : action_server_(*node_handler, action_server_name, false)
    {
        action_server_.registerGoalCallback(boost::bind(&GetClosestPointServer::goal_callback, this));
        action_server_.registerPreemptCallback(boost::bind(&GetClosestPointServer::preempt_callback, this));
        action_server_.start();
    }

    /**
      Callback executed when a goal is received. Your code should be called or written inside this method
     */
    void GetClosestPointServer::goal_callback()
    {
        // If the server is already processing a goal
        if (busy_)
        {
            ROS_ERROR("The action will not be processed because the server is already busy with another action. "
                      "Please preempt the latter or wait before sending a new goal");
            return;
        }
        // The first step is to accept a new goal. If you want to access to the input field, you should write
        // new_goal_->input;
        new_goal_ = action_server_.acceptNewGoal();
        // Set busy to true
        busy_ = true;

        geometry_msgs::Point end_effector_position = new_goal_->input;

        // Create a vector containing the values of the position of the end effector set as input
        std::vector<double> ee_pos{ end_effector_position.x, end_effector_position.y, end_effector_position.z};
        // At first we want min_dist to be a large number
        float min_distance = 1000.0;
        // Will store the point that is the closest to the end effector position
        std::vector<double> min_point;
        //For each point we can send a feedback
        for (int index = 0; index < WAYPOINTS.size(); index++)
        {
            double distance = sqrt(pow(ee_pos[0] - WAYPOINTS[index][0], 2) + pow(ee_pos[1] - WAYPOINTS[index][1], 2) + pow(ee_pos[2] - WAYPOINTS[index][2], 2));
            if (distance < min_distance)
            {
                min_distance = distance;
                min_point = WAYPOINTS[index];
            }
            // Set the current point currently tested
            action_feedback_.current_point = WAYPOINTS[index];
            // Set the current minimum found
            action_feedback_.min_distance = min_distance;
            // Publish the feedback
            action_server_.publishFeedback(action_feedback_);
        }

        // Now that the loop is done, create a Point msg
        geometry_msgs::Point output_msg;
        output_msg.x = min_point[0];
        output_msg.y = min_point[1];
        output_msg.z = min_point[2];

        // Return the response of the action.
        action_result_.outcome = 0;
        // We only have one possible outcome here, so always send 0
        action_result_.returned_object = output_msg;
        action_server_.setSucceeded(action_result_);
        // Set busy to false
        busy_ = false;
    }

    void GetClosestPointServer::preempt_callback()
    {
        ROS_INFO("Action preempted");
        action_server_.setPreempted();
    }

    int main(int argc, char** argv)
    {
        // Initialise the node with a specific name (please change it to match your action)
        ros::init(argc, argv, "get_closest_cpp_action_server");
        // Initialise the node handler that will be given to the class' constructor
        ros::NodeHandle node_handler;
        /**
        Create an instance of the class running the action server. The argument of the class determines the name of the
        action. Make sure to change it to match your code.
        */
        GetClosestPointServer closes_point_server_(&node_handler, "get_closest_point");
        // Wait for shutdown
        ros::spin();
        return 0;
    }

5. Make sure the server is executable
#####################################

5.1 For Python
**************
If you have created your server from :code:`template_action_server.py`, be sure to make your new file executable with :code:`chmod +x`:

.. prompt:: bash $

    chmod +x /home/user/projects/shadow_robot/base/src/example_package/scripts/get_closest_point_server.py

Otherwise, ROS won't be able to locate your node.

5.2 For C++
***********
In order to make ROS aware of your newly created server, we need to slightly modify :code:`CMakeLists.txt`. In the **Build** section of this file (you can go to line 167), add these three lines for **each** cpp service file you have created:

.. code-block:: CMake

    add_executable(<node_name> scripts/<cpp_file_name>)
    target_link_libraries(<node_name> ${catkin_LIBRARIES})
    add_dependencies(<node_name> <package_name>)

In our case, we would have:

.. code-block:: CMake

    add_executable(get_closest_cpp_action_server scripts/get_closest_point_server.cpp)
    target_link_libraries(get_closest_cpp_action_server ${catkin_LIBRARIES})
    add_dependencies(get_closest_cpp_action_server example_package)

5.3 Common step
***************
The last thing you need to do is to recompile your ROS package:

.. prompt:: bash $

    cd /home/projects/shadow_robot/base
    catkin_make
    source devel/setup.bash

And here you are! You have successfully wrapped your code in a ROS action **fully compatible** with GRIP!
