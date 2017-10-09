## Writing Ros nodes

###1. Ros publishers
pub1 = rospy.Publisher("/topic_name", message_type, queue_size=size)
pub1.publish(message)

###2. Create a node simple_mover
- To command joint movement for simple_arm

cd ~/catkin_ws/src/simple_arm/
mkdir scripts

###2.1 Create new scripts
cd scripts
echo '#!/bin/bash' >> hello
echo 'echo Hello World' >> hello
chmod u+x hello
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun simple_arm hello

###Creating the empty simple_mover node script
cd ~/catkin_ws/src/simple_arm
cd scripts
touch simple_mover
chmod u+x simple_mover

##Simple Mover Code
Assuming that your workspace has recently been built, and it’s setup.bash has been sourced, you can launch simple_arm as follows:
cd ~/catkin_ws
roslaunch simple_arm robot_spawn.launch

Once ROS Master, Gazebo, and all of our relevant nodes are up and running, we can finally launch simple_mover. To do so, open a new terminal and type the following commands:

cd ~/catkin_ws
source devel/setup.bash
rosrun simple_arm simple_mover

###Ros Services
service = rospy.Service('service_name', serviceClassName, handler)


service_proxy = rospy.ServiceProxy('service_name', serviceClassName)
Example
msg = serviceClassNameResponse()
#update msg attributes here to have correct data
response = service_proxy(msg)

### Creating A Arm Mover Service

As you learned earlier, an interaction with a service consists of two messages being passed. A request passed to the service, and a response received from the service. The definitions of the request and response message type are contained within .srv files living in the srv directory under the package’s root.

Let’s define a new service for simple_arm. We shall call it GoToPosition.

cd ~/catkin_ws/src/simple_arm/
mkdir srv
cd srv
touch GoToPosition.srv

Edit GoToPosition.src, add the content

float64 joint_1
float64 joint_2
---
duration time_elapsed

Service definitions always contain two sections, separated by a ‘---’ line. The first section is the definition of the request message. Here, a request consists of two float64 fields, one for each of simple_arm’s joints. The second section contains is the service response. The response contains only a single field, time_elapsed. The time_elapsed field is of type duration, and is responsible for indicating how long it took the arm to perform the movement.

Note: Defining a custom message type is very similar, with the only differences being that message definitions live within the msg directory of the package root, have a “.msg” extension, rather than .srv, and do not contain the “---” section divider. You can find more detailed information on creating messages and services here, and here, respectively.


### Modifying CMakeLists.txt

In order for catkin to generate the python modules or C++ libraries which allow you to utilize messages in your code you must first modify simple_arm’s CMakeLists.txt (~/catkin_ws/src/simple_arm/CMakeLists.txt).

CMake is the build tool underlying catkin, and CMakeLists.txt is nothing more than a CMake script used by catkin. If you’re familiar with GNU make, and the concept of makefiles, this is a similar concept.

First, ensure that the find_package() macro lists std_msgs and message_generation as required packages. The find_package() macro should look as follows:

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation
)

As the names might imply, the std_msgs package contains all of the basic message types, and message_generation is required to generate message libraries for all the supported languages (cpp, lisp, python, javascript).

Note: In your CMakeLists.txt, you may also see controller_manager listed as a required package. In actuality this package is not required. It was simply added as a means to demonstrate a build failure in the previous lesson. You may remove it from the list of REQUIRED COMPONENTS if you choose.

Next, uncomment the commented-out add_service_files() macro so it looks like this:

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GoToPosition.srv
)


This tells catkin which files to generate code for.

Lastly, make sure that the generate_messages() macro is uncommented, as follows:

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

### Modifying package.xml

Now that the CMakeLists.txt file has been covered, you should technically be able to build the project. However, there’s one more file which needs to be modified, package.xml.

package.xml is responsible for defining many of the package’s properties, such as the name of the package, version numbers, authors, maintainers, and dependencies.

Right now, we’re worried about the dependencies. In the previous lesson you learned about build-time dependencies and run-time package dependencies. When rosdep is searching for these dependencies, it’s the package.xml file that is being parsed. Let’s add the message_generation and message_runtime dependencies.


 <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>

  <run_depend>controller_manager</run_depend>
  <run_depend>effort_controllers</run_depend>
  <run_depend>gazebo_plugins</run_depend>
  <run_depend>gazebo_ros</run_depend>
  <run_depend>gazebo_ros_control</run_depend>
  <run_depend>joint_state_controller</run_depend>
  <run_depend>joint_state_publisher</run_depend>
  <run_depend>robot_state_publisher</run_depend>
  <run_depend>message_runtime</run_depend>
  <run_depend>xacro</run_depend>

### Building the package

cd ~/catkin_ws
catkin_make
cd devel/lib/python2.7/dist-packages
ls

env | grep PYTHONPATH

### Creating the empty arm_mover node script
cd ~/catkin_ws
cd src/simple_arm/scripts
touch arm_mover
chmod u+x arm_mover

### Launching the project with a new service
To get the arm_mover node, and accompanying safe_move service to launch along with all of the other nodes, you will modify robot_spawn.launch.

Launch files, when they exist, are located within the launch directory in the root of a catkin package. simple_arm’s launch file is located in ~/catkin_ws/src/simple_arm/launch

To get the arm_mover node to launch, simply add the following:

 <!-- The arm mover node -->
  <node name="arm_mover" type="arm_mover" pkg="simple_arm">
    <rosparam>
      min_joint_1_angle: 0
      max_joint_1_angle: 1.57
      min_joint_2_angle: 0
      max_joint_2_angle: 1.0
    </rosparam>
  </node>



### Testing the new service

Now that you've modified the launch file, you are ready to test everything out.

To do so, launch the simple_arm, verify that the arm_mover node is running, and that the safe_move service is listed:

Note: You will need to make sure that you've exited out of your previous roslaunch session before re-launching.

cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch simple_arm robot_spawn.launch
rosnode lists
rosservice list
rqt_image_view /rgb_camera/image_raw

rosservice call /arm_mover/safe_move "joint_1: 1.57
joint_2: 1.57"

rosparam set /arm_mover/max_joint_2_angle 1.57

rosservice call /arm_mover/safe_move "joint_1: 1.57
joint_2: 1.57"

References
http://docs.ros.org/kinetic/api/rospy/html/rospy.topics.Publisher-class.html
http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
http://docs.ros.org/kinetic/api/rospy/html/
http://wiki.ros.org/rospy/Overview/Services
http://wiki.ros.org/msg
http://wiki.ros.org/srv
http://wiki.ros.org/roslaunch/XML