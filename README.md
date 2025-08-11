# turtlebot3_formation
Leader-follower formation control using ROS+Gazebo. 

## Introduction
This repository use 4 robots (turtlebot3) formation control. The main idea is: one turtlebot is leader, the remaining three turtlebots are follower. The followers use PID control algorithm to maintain the pose and distance with leader robot. More detail is on [abc_swarm doc](https://abc-swarm.readthedocs.io/en/latest/index.html). 

![System image](include/four_tb3_formation.png "4 Turtlebot3 formation control in Gazebo")

## Getting Started
1. Create a ROS workspace
```console
$ mkdir <workspace_name>/src
$ cd <workspace_name>/src
```

2. install dependencies and repository
```console
$ git clone <dependency repository>.git
$ git clone git@github.com:khinggan/turtlebot3_formation.git
```
dependencies: `turtlebot3_msg`, `turtlebot3`, `turtlebot3_simulation`, `dynamic_reconfigure`.

3. build
```console
$ cd ../
$ catkin_make
$ source devel/setup.bash
```

4. run
```console
$ roslaunch turtlebot3_formation leader_follower.launch
```

5. test (another terminal)
```console
$ rostopic pub -r 10 /tb3_1/cmd_vel geometry_msgs/Twist "{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.1}}"
```

### BUG [#3](https://github.com/khinggan/turtlebot3_formation/issues/3)
> Modify the [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3/blob/noetic/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro) noetic branch, `turtlebot3_burger.gazebo.xacro` file, 
> 
```xml
  ...
  <xacro:arg name="laser_visual" default="false"/>
  <xacro:arg name="imu_visual"   default="false"/>

  <!--Add this-->
  <xacro:arg name="tf_prefix" default=""/>
  ...
```
and use tf_prefix in gazebo plugin

```xml
  <gazebo>
    <plugin name="turtlebot3_burger_controller" filename="libgazebo_ros_diff_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>$(arg tf_prefix)/odom</odometryFrame>
      <odometrySource>world</odometrySource>
      <publishOdomTF>true</publishOdomTF>
      <robotBaseFrame>$(arg tf_prefix)/base_footprint</robotBaseFrame>
      <publishWheelTF>false</publishWheelTF>
      <publishTf>true</publishTf>
      <publishWheelJointState>true</publishWheelJointState>
      <legacyMode>false</legacyMode>
      <updateRate>30</updateRate>
      <leftJoint>wheel_left_joint</leftJoint>
      <rightJoint>wheel_right_joint</rightJoint>
      <wheelSeparation>0.160</wheelSeparation>
      <wheelDiameter>0.066</wheelDiameter>
      <wheelAcceleration>1</wheelAcceleration>
      <wheelTorque>10</wheelTorque>
      <rosDebugLevel>na</rosDebugLevel>
    </plugin>
  </gazebo>

```