## Logging Files

By default all logging messages for a node are written to the node's log file which can be found in ~/.ros/log or ROS_ROOT/log . If roscore is running, you can use roscd to find log file directory by opening a new terminal window and typing:


ROS_ROOT/log 
~/.ros/log

or 

roscd log


### Filtering and Saving Log Messages from /rosout
rostopic echo /rosout
rostopic echo /rosout | grep insert_search_expression_here

### Modifying message level sent to /rosout
Although logdebug messages are not written to /rosout by default, it is possible to modify the level of logging messages written to /rosout to display them there, or change the level of logging messages written to /rosout to be more restrictive. To do this you must set the log_level attribute within the rospy.init_node code. For example, if you'd like to allow lodebug messages to be written to /rosout, that can be done as follows:

rospy.init_node('my_node', log_level=rospy.DEBUG)

## Modifying display of messages sent to stdout and stderr
         stdout stderr   
"screen" screen screen
"log"    log  screen and log

Eg:  <!-- The look away node -->
  <node name="look_away" type="look_away" pkg="simple_arm" output="screen"/>