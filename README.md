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

