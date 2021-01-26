.. _ros_services:
*********************
Creating ROS services
*********************

| This page provides a step-by-step guide to create a ROS service from already existing code. Note that this tutorial includes some specific steps that ensures your service can be used by GRIP!
|
| Let's consider that you have a script that randomly generates the position of the end-effector of the robot within a given area. To do so, let's assume that you have the following python code:

.. code-block:: python

    #!/usr/bin/env python

    import random

    def generate_ee_position(min_x, max_x, min_y, max_y, min_z, max_z):
        """
            Generate a random position within the input area
        """
        x_value = random.uniform(min_x, max_x)
        y_value = random.uniform(min_y, max_y)
        z_value = random.uniform(min_z, max_z)

        return [x_value, y_value, z_value]

The C++ equivalent of this code would be the following:

.. code-block:: cpp

    #include <random>
    #include <vector>

    // You can define here as many functions as you want

    float random_float(float min, float max)
    {
        // this  function assumes max > min, you may want
        // more robust error checking for a non-debug build
        assert(max > min);
        float random = ((float) rand()) / (float) RAND_MAX;

        float range = max - min;
        return (random * range) + min;
    }

    std::vector<float> generate_ee_position(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
    {
        float x_value = random_float(min_x, max_x);
        float y_value = random_float(min_y, max_y);
        float z_value = random_float(min_z, max_z);

        std::vector<float> output_vector{ x_value, y_value, z_value };

        return output_vector;
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

3. Create msg and srv files
###########################
| The **srv** file will contain the backbone of what should be received and sent by the server. In order to be fully compatible with GRIP, the srv file **must** respect the following format:

.. code-block:: bash

    <msg_type> input
    ---
    int8 outcome
    <msg_type> returned_object

| :code:`<msg_type>` is left to your preference, i.e. it's implementation-dependent. You can use all the `built-in types <http://wiki.ros.org/msg>`_ or :ref:`create your own msg file <create_msg>`.
| In our case, the function we want to wrap inside a ROS service expects 6 values as input and outputs 3 values. For this reason, we are going to create a new file, :code:`GeneratePosition.srv`, in the :code:`srv` folder with the following content

.. code-block:: bash

    float32[] input
    ---
    int8 outcome
    float32[] returned_object

| Before forgetting, let's edit the :code:`CMakeLists.txt` file. Go to line 67 and add the name of your srv file after :code:`FILES` (if you have several, add one per line). In our case, we would get

.. code-block:: CMake

    add_service_files(
      FILES
      GeneratePosition.srv
    )

| To make sure everything works so far, run

.. prompt:: bash $

    cd /home/user/projects/shadow_robot/base
    catkin_make
    source devel/setup.bash

4. Filling the templates
########################
| If the code you want to wrap inside ROS services is written in C++, you will find the :code:`template_service_server.cpp` file in :code:`scripts`. If you are using python, the template file is :code:`template_service_server.py`. Although the syntaxes are different, the steps are exactly the same (and described in the files). You can either copy/paste and rename these files or just rename them. In our case we are going to create the file :code:`generate_ee_position_server.py` (python version) and :code:`generate_ee_position_server.cpp` (C++ version).

4.1 Import the generated srv files
**********************************
Make sure to change the occurences of :code:`server_template` by your package name (e.g. :code:`example_package`). Then, change :code:`srvName` by the name of your srv file without the extension (e.g. :code:`GeneratePosition`).

4.2 Add your code
*****************
| Add your code in the file. It can either directly be inside the function :code:`my_function` (feel free to change its name), or create a new function, class or whatever you prefer.
| **In Python**, if you want to access the values stored in the :code:`input` field of your srv file inside :code:`my_function`, you can use :code:`input_values = request.input` and carry out any operation on the variable :code:`input_values`.
| Similarly, **in C++**, you can use :code:`<variable_type> input_values = request.input;`.

4.3 Return the result
*********************
Make sure to fill and return the response of your server, keeping in mind that it must contain the fields :code:`outcome` and :code:`returned_object`.

4.4 Call your code when the script is executed
**********************************************
Change the name of the ROS node that will run your server (string inside :code:`rospy.init_node()` or :code:`ros::init()`). The last step is to make sure its content is valid. The first argument sets the **name of the service**, the second one (not in C++) should be the **name of your srv file** and the last one should be the **name of the function returning the server response**.

In our example, the :code:`generate_ee_position_server.py` looks like this:

.. code-block:: python

    #!/usr/bin/env python

    import rospy
    # Change server_template by the name of your ROS package and srvName by the name of your srv file (without the .srv)
    from example_package.srv import GeneratePosition, GeneratePositionResponse

    # You can add here any other import statement you might need for your code
    import random


    # You can define here as many functions as you want
    def generate_ee_position(min_x, max_x, min_y, max_y, min_z, max_z):
        """
            Generate a random position within the input area
        """
        x_value = random.uniform(min_x, max_x)
        y_value = random.uniform(min_y, max_y)
        z_value = random.uniform(min_z, max_z)

        return [x_value, y_value, z_value]


    # Function that is going to be linked to the service server
    def randomly_generate_end_effector_position(request):
        """
            This function (feel free to change its name) must contain your code.
            It MUST have a single input argument, that will be the request part of the srv, so DO NOT change it.
            You can call other functions inside it without any problem
        """
        # Change srvName by the name of your srv
        response = GeneratePositionResponse()

        # Get the input list
        input_values = request.input
        # Make sure the input list has 6 values, otherwise fill the response with a negative outcome (indexed as 1 here)
        if len(input_values) != 6:
            response.outcome = 1
            # Return an empty list
            response.returned_object = list()
        else:
            response.returned_object = generate_ee_position(input_values[0], input_values[1], input_values[2],
                                                            input_values[3], input_values[4], input_values[5])
            # Success outcome
            response.outcome = 0
        # Return the response
        return response

    if __name__ == '__main__':
        # Initialise the node with a specific name (please change it to match your service)
        rospy.init_node('python_generate_ee_position_service_server')
        # Set the name of the service, specify which kind of srv will trigger it and what function will be run.
        # Change the name of the server with one that matches the content of your code, set the second argument to the name
        # of the srv file, and the last one should be the name of the function that runs your code.
        rospy.spin()

Similarly, here is the content of :code:`generate_ee_position_server.cpp`:

.. code-block:: cpp

    #include <ros/ros.h>
    /**
    Change server_template by the name of your ROS package and srvName by the name of your srv file (without the .srv)
    */
    #include <example_package/GeneratePosition.h>

    // You can add here any other include statement you might need for your code
    #include <random>
    #include <vector>

    // You can define here as many functions as you want

    float random_float(float min, float max)
    {
        // this function assumes max > min, you may want more robust error checking for a non-debug build
        assert(max > min);
        float random = ((float) rand()) / (float) RAND_MAX;

        float range = max - min;
        return (random * range) + min;
    }

    std::vector<float> generate_ee_position(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
    {
        float x_value = random_float(min_x, max_x);
        float y_value = random_float(min_y, max_y);
        float z_value = random_float(min_z, max_z);

        std::vector<float> output_vector{ x_value, y_value, z_value };

        return output_vector;
    }

    // Function that is going to be linked to the service server
    // This function (feel free to change its name) must contain your code.
    // It MUST have two paramters, one for the request the srv, and the other one for the response.
    // You can call other functions inside it without any problem
    bool randomly_generate_end_effector_position(example_package::GeneratePosition::Request &request,
                                                 example_package::GeneratePosition::Response &response)
    {
      // Get the input list
      std::vector<float> input_values = request.input;

      // Make sure the input list has 6 values, otherwise fill the response with a negative outcome (indexed as 1 here)
      if (input_values.size() != 6)
      {
        response.outcome = 1;
        // Return an empty list
        std::vector<float> empty_vector;
        response.returned_object = empty_vector;
      }
      else
      {
        response.returned_object = generate_ee_position(input_values[0], input_values[1], input_values[2],
                                                        input_values[3], input_values[4], input_values[5]);
        // Success outcome
        response.outcome = 0;
      }

      // Return true to avoid having runtime errors on the client side
      return true;
    }

    int main(int argc, char **argv)
    {
      // Initialise the node with a specific name (please change it to match your service)
      ros::init(argc, argv, "cpp_generate_ee_position_service_server");
      // Create a node handler
      ros::NodeHandle node_handle;
      /**
      Set the name of the service and what function will be run and received
      Change the name of the server with one that matches the content of your code, and the last one should be
      the function that runs your code.
      */
      ros::ServiceServer service = node_handle.advertiseService("ee_position_generation", randomly_generate_end_effector_position);
      ros::spin();

      return 0;
    }

5. Make sure the server is executable
#####################################

5.1 For Python
**************
If you have created your server from :code:`template_service_server.py`, be sure to make your new file executable with :code:`chmod +x`:

.. prompt:: bash $

    chmod +x /home/user/projects/shadow_robot/base/src/example_package/scripts/generate_ee_position_server.py

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

    add_executable(cpp_generate_ee_position_service_server scripts/generate_ee_position_server.cpp)
    target_link_libraries(cpp_generate_ee_position_service_server ${catkin_LIBRARIES})
    add_dependencies(cpp_generate_ee_position_service_server example_package)

5.3 Common step
***************
The last thing you need to do is to recompile your ROS package:

.. prompt:: bash $

    cd /home/projects/shadow_robot/base
    catkin_make
    source devel/setup.bash

And here you are! You have successfully wrapped your code in a ROS service **fully compatible** with GRIP!
