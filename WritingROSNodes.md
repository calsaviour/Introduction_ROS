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

References
http://docs.ros.org/kinetic/api/rospy/html/rospy.topics.Publisher-class.html
http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
http://docs.ros.org/kinetic/api/rospy/html/
http://wiki.ros.org/rospy/Overview/Services