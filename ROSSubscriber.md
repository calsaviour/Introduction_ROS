### ROS Subscribers
sub1 = rospy.Subscriber("/topic_name", message_type, callback_function)

The "/topic_name" indicates which topic the Subscriber should listen to.

The message_type is the type of message being published on "/topic_name".

The callback_function is the name of the function that should be called with each incoming message. Each time a message is received, it is passed as an argument to callback_function. Typically, this function is defined in your node to perform a useful action with the incoming data. Note that unlike service handler functions, the callback_function is not required to return anything.

### Description of Look Away
To see a Subscriber in action, you'll write a node called look_away. The look_away node will subscribe to the /rgb_camera/image_raw topic, which has image data from the camera mounted on the end of the robotic arm. Whenever the camera is pointed towards an uninteresting image - in this case, an image with uniform color - the callback function will move the arm to something more interesting. There are a few extra pieces in the code to ensure that this procedure is executed smoothly, but you will learn more about them later.

### Creating the empty look_away node script
cd ~/catkin_ws
cd src/simple_arm/scripts
touch look_away
chmod u+x look_away

## Updating the launch file

<!-- The look away node -->
  <node name="look_away" type="look_away" pkg="simple_arm"/>

   <!-- The arm mover node -->
  <node name="arm_mover" type="arm_mover" pkg="simple_arm">
    <rosparam>
      min_joint_1_angle: 0
      max_joint_1_angle: 1.57
      min_joint_2_angle: 0
      max_joint_2_angle: 1.57
    </rosparam>
  </node>
  
References
http://docs.ros.org/api/rospy/html/rospy.topics.Subscriber-class.html