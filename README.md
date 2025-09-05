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




