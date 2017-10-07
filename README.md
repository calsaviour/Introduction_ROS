## Introduction To ROS

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

Start Robot Master
roscore

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

