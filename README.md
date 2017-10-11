## Introduction To ROS

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

Start Robot Master
```
bash roscore
```

vm password : udacity-nd

Start node
rosrun <package> <node>
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key


source /opt/ros/kinetic/

1. List Running Node
rosnode list

2. List topic
rostopic list

3. List topic info
rostopic info /turtle1/cmd_vel

4. Info of message type
rosmsg show geometry_msgs/Twist

5. Look into the source of the message type
rosed geometry_msgs Twist.msg

6. Echo a topic
rostopic echo /turtle1/cmd_vel


Reference
https://ll.mit.edu/publications/technotes/TechNote_LGPR.pdf
http://www.mdpi.com/1424-8220/16/3/280/htm
http://wiki.ros.org/ROS/Tutorials/UsingRosEd


## Packages and Catkin Workspaces
Catkin is a package management tool

1. Create a catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
ls -l
cd ~/catkin_ws
catkin_make

Reference:
http://wiki.ros.org/catkin/conceptual_overview
http://www.ros.org/reps/rep-0128.html


2. Add a package
cd ~/catkin_ws/src
git clone https://github.com/udacity/simple_arm_01.git simple_arm
cd ~/catkin_ws
catkin_make
sudo apt-get install ros-kinetic-controller-manager
catkin_make

3. Roslaunch
roslaunch allows you to do the following
Launch ROS Master and multiple nodes with one simple command
Set default parameters on the parameter server
Automatically re-spawn processes that have died

cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch simple_arm robot_spawn.launch

4. Getting packages
source devel/setup.bash
rosdep check simple_arm
rosdep install -i simple_arm or sudo apt-get ros-kinetic-gazebo-ros-control

5. Dive Deeper into Packages

cd ~/catkin_ws/src
catkin_create_pkg <your_package_name> [dependency1 dependency2 …]
catkin_create_pkg first_package

Reference:
http://www.ros.org/browse/list.php
http://wiki.ros.org/
https://cse.sc.edu/~jokane/agitr/
http://answers.ros.org/