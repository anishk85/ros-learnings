# ros-learnings
this file contains usually errors which i encountered in ros and fixed them 

# make sure you add dummy link in the urdf or xacro file otherwise at some points it doent load in rviz or gazebo 
```bash
<?xml version="1.0"?> 
<robot name="new_lifting_mechanism_final" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="dummy_link"/>
  
  <joint name="dummy_joint" type="fixed">
    <parent link="dummy_link"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <xacro:include filename="lifting_mechanism_core.xacro"/>
  <xacro:include filename="gazebo_control.xacro"/>
  <xacro:include filename="ros2_control.xacro"/>

</robot>
```
also define this in rviz file 
```
Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: dummy_link
    Frame Rate: 30
```
you need a fixed frame 

#sometimes like your paths in urdf files for meshes or stl doesn't works well 
https://robotics.stackexchange.com/questions/111580/rviz2-can-not-load-resource-of-mesh-geometry
here's the issue which addresses this 
```bash
change these to 
<mesh filename="package://my_package/meshes/rp_lidar.dae"/>
to 
<mesh filename="$(find my_package)/meshes/rp_lidar.dae"/>
now gazebo loads this but rviz gave error retrieving file so here is the fix 

chnage the all file paths to this 
<mesh filename="file://$(find my_package)/meshes/rp_lidar.dae"/>
thsi is the final fix which worked for me 
this is for ros2 humble gazebo classic i hope this works for all other version it's one of the most common error i encountered 
```

#sometimes i faced the ros2 run no executable found so there's might be an issue that your cmake lists.txt has some error 
like i did this 
```bash
# Install directories
install(DIRECTORY
  config
  launch
  meshes
  urdf
  scripts

#   worlds
  DESTINATION share/${PROJECT_NAME}/
)
```
but the correct fix was this
``` bash
install(PROGRAMS
  scripts/teleop_robot.py
  DESTINATION lib/${PROJECT_NAME}
)
```
#Setting up moveit servo
robot description error 
urdf did not loaded 
so first you need to craete a servo_server then i copied my robot decription into that 
``` bash
#!/usr/bin/env python3
import rospy

def copy_robot_description():
    rospy.init_node('copy_robot_desc')
    
    # Copy robot description from my_gen3 namespace to servo_server namespace
    if rospy.has_param('/my_gen3/robot_description'):
        robot_desc = rospy.get_param('/my_gen3/robot_description')
        rospy.set_param('/servo_server/robot_description', robot_desc)
        print("✅ Robot description copied successfully!")
    else:
        print("❌ Robot description not found!")

if __name__ == '__main__':
    copy_robot_description()
```
then for kinova robotic arm i encountered that i there was mismatch of mesaage type so i craeted velocity bridge 

```bash
# This revealed the real problem!
rostopic info /my_gen3/in/joint_velocity
# Type: std_msgs/Float64MultiArray
# Subscribers: * /my_gen3/my_gen3_driver

# But when we tried to publish:
rostopic pub -r 10 /my_gen3/in/joint_velocity std_msgs/Float64MultiArray "data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
# ERROR: topic types do not match: [kortex_driver/Base_JointSpeeds] vs. [std_msgs/Float64MultiArray]
find /root/ros_ws/src/niwesh/kinova_urc_arm -name "*.cpp" -o -name "*.py" | xargs grep -l "joint_velocity"
grep -A 10 -B 5 "joint_velocity" /root/ros_ws/src/niwesh/kinova_urc_arm/kortex_driver/src/non-generated/driver/kortex_subscribers.cpp
```

velocity bridge 
``` bash
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import importlib
import sys

# Try to import the correct message type
try:
    # Try different possible locations for the message
    from kortex_driver.msg import Base_JointSpeeds, JointSpeed
    print("Found Base_JointSpeeds in kortex_driver.msg")
except ImportError:
    try:
        # Search in generated messages
        sys.path.append('/root/ros_ws/src/niwesh/kinova_urc_arm/kortex_driver/src/generated')
        from kortex_driver.msg import Base_JointSpeeds, JointSpeed
        print("Found Base_JointSpeeds in generated")
    except ImportError:
        print("ERROR: Cannot find Base_JointSpeeds message type")
        # Let's see what's available
        rospy.logfatal("Cannot import Base_JointSpeeds. Available message types:")
        import kortex_driver.msg as kdm
        print(dir(kdm))
        sys.exit(1)

class VelocityBridge:
    def __init__(self):
        rospy.init_node('velocity_bridge')
        
        # Subscribe to servo output
        rospy.Subscriber('/servo_server/command', Float64MultiArray, self.servo_callback)
        
        # Publish to robot using correct message type
        self.joint_vel_pub = rospy.Publisher('/my_gen3/in/joint_velocity', Base_JointSpeeds, queue_size=1)
        
        rospy.loginfo("Velocity bridge initialized - converting Float64MultiArray to Base_JointSpeeds")
        
    def servo_callback(self, msg):
        """Convert Float64MultiArray to Base_JointSpeeds"""
        joint_speeds = Base_JointSpeeds()
        joint_speeds.joint_speeds = []
        
        # Convert each velocity to JointSpeed message
        for i, velocity in enumerate(msg.data):
            if i < 7:  # Only use first 7 joints (arm joints, not gripper)
                joint_speed = JointSpeed()
                joint_speed.joint_identifier = i
                joint_speed.value = float(velocity)
                joint_speeds.joint_speeds.append(joint_speed)
        
        # Publish to robot
        self.joint_vel_pub.publish(joint_speeds)
        rospy.logdebug(f"Published joint velocities: {[js.value for js in joint_speeds.joint_speeds]}")

if __name__ == '__main__':
    try:
        bridge = VelocityBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal(f"Bridge failed: {e}")
```



Your URDF file likely references meshes like this: package://rover_description/meshes/some_mesh.STL.

For Gazebo to understand what package://rover_description means, its GAZEBO_RESOURCE_PATH environment variable must point to the directory containing the rover_description package.

Your package is at /root/catkin_ws/src/rover_description.

Therefore, GAZEBO_RESOURCE_PATH must contain /root/catkin_ws/src, which is equivalent to $(find rover_description)/...

Your launch file correctly sets GAZEBO_MODEL_PATH but incorrectly sets GAZEBO_RESOURCE_PATH.




