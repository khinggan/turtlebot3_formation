#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point
from visualization_msgs.msg import Marker
import numpy as np
import math
import tf

# ** Author: khinggan ** 
# ** Email: khinggan2013@gmail.com **

"""Description 
Four robot formation control using virtual structure algorithm https://link.springer.com/article/10.1023/A:1008814708459
"""

## Turtlebot3 Parameters
MAX_LINEAR_SPEED = 0.15
MAX_ANGULAR_SPEED = 1.5


class VirtualStructure:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('virtual_structure', anonymous=True)

        # 4 robot current poses (position, orientation)
        self.robot_pose_1 = Pose()
        self.robot_pose_2 = Pose()  
        self.robot_pose_3 = Pose()  
        self.robot_pose_4 = Pose()  
        
        self.WaypointType = "ellipse"    # Waypoints type: ellipse or sin
        self.waypoints = self.get_waypoints(self.WaypointType)
        
        self.distance = 1.0           # get from parameter
        self.reach_threshold = 1.0    # [m] # Reach threshold, sum of formation error is lower then this, move to next target virtual structure

        # Pure Pursuit Algorithm Parameters
        self.Kp_rho = 3
        self.Kp_alpha = 8
        self.Kp_beta = -1.5
        self.lookahead_distance = 0.3   # [m] Pure Pursuit Algorithm Parameters
        self.old_nearest_point_index = None
        self.last_index = len(self.waypoints[0]) - 1
        self.target_idx = 0    # target index, target virtual structure can be got from this. 
        self.target_angle = 0.0
        
        # Create odom subscriber tb3_1~tb3_4
        self.odom_subscriber_1 = rospy.Subscriber('/tb3_1/odom', Odometry, self.odom_callback_1)
        self.odom_subscriber_2 = rospy.Subscriber('/tb3_2/odom', Odometry, self.odom_callback_2)
        self.odom_subscriber_3 = rospy.Subscriber('/tb3_3/odom', Odometry, self.odom_callback_3)
        self.odom_subscriber_4 = rospy.Subscriber('/tb3_4/odom', Odometry, self.odom_callback_4)
        
        # Create a velocity publisher tb3_1~tb3_4
        self.velocity_publisher_1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.velocity_publisher_2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
        self.velocity_publisher_3 = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)
        self.velocity_publisher_4 = rospy.Publisher('/tb3_4/cmd_vel', Twist, queue_size=10)

        # Path Visualization publisher
        self.waypoints_publisher = rospy.Publisher("waypoints", Marker, queue_size=10)
        
        # Set a rate for publishing messages
        self.rate = rospy.Rate(10) # 10 Hz
    
    def odom_callback_1(self, data):
        self.robot_pose_1 = data.pose.pose
    def odom_callback_2(self, data):
        self.robot_pose_2 = data.pose.pose
    def odom_callback_3(self, data):
        self.robot_pose_3 = data.pose.pose
    def odom_callback_4(self, data):
        self.robot_pose_4 = data.pose.pose

    def get_waypoints(self, waypoint_type):
        '''
        waypoint_type: `sin` or `ellipse` path waypoints
        ellipse: 2a = 20, 2b = 10
        sin: long = 20, width = 13
        return: path X (list), path Y (list)

        self.lookahead_distance should > nearest two waypoint distance.
        the density of the waypoints are affect the path tracking performance

        sin: path_X = np.arange(0, 20, 0.1); 0.1 affect density
        ellipse: thetas = np.linspace(-np.pi, np.pi, 200); 200 affect density

        '''
        path_X, path_Y = None, None
        if waypoint_type == "sin":
            path_X = np.arange(0, 20, 0.1)
            path_Y = [math.sin(ix / 2.0) * ix / 2.0 for ix in path_X]
        elif waypoint_type == "ellipse":
            thetas = np.linspace(-np.pi, np.pi, 200)
            a, b = 2, 1.3
            path_X = [a * np.cos(theta) + a for theta in thetas]
            path_Y = [b * np.sin(theta) for theta in thetas]
        else:
            path_X = [0.0]
            path_Y = [0.0]
            print("{} TYPE of waypoints not exist".format(waypoint_type))
        return path_X, path_Y

    def get_target_point(self):
        # Parameters: current position, look ahead distance
        # Return: target_point x, y, z
        cx = self.robot_pose_1.position.x
        cy = self.robot_pose_1.position.y

        if self.old_nearest_point_index is None:
            dx = [cx - icx for icx in self.waypoints[0]]
            dy = [cy - icy for icy in self.waypoints[1]]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = math.hypot(self.waypoints[0][ind] - cx,
                                            self.waypoints[1][ind] - cy)
            while True:
                if (ind + 1) >= len(self.waypoints[0]):
                    break
                distance_next_index = math.hypot(self.waypoints[0][ind + 1] - cx,
                                                self.waypoints[1][ind + 1]- cy)
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.waypoints[0]) else ind
                distance_this_index = distance_next_index
                
            self.old_nearest_point_index = ind
        # Lf = k * self.twist.linear.x + Lfc  # use Changing look ahead distance
        Lf = self.lookahead_distance          # or use Fixed size look ahead distance

        # search look ahead target point index
        while Lf > math.hypot(self.waypoints[0][ind] - cx, self.waypoints[1][ind] - cy):
            if (ind + 1) >= len(self.waypoints[0]):
                break  # not exceed goal
            ind += 1

        self.target_idx = ind

        # get target angle
        dy = self.waypoints[1][self.target_idx] - cx
        dx = self.waypoints[0][self.target_idx] - cy
        self.target_angle = math.atan2(dy, dx)

        return self.waypoints[0][self.target_idx], self.waypoints[1][self.target_idx]

    def get_own_target(self, tx, ty):
        tx_1, ty_1, tx_2, ty_2, tx_3, ty_3, tx_4, ty_4 = tx, ty, tx - 1, ty, tx - 1, ty - 1, tx, ty - 1
        return tx_1, ty_1, tx_2, ty_2, tx_3, ty_3, tx_4, ty_4

    def virtual_structure(self):
        twist_1 = Twist()
        twist_2 = Twist()
        twist_3 = Twist()
        twist_4 = Twist()
        while not rospy.is_shutdown():
            # Get target point on waypoints
            tx, ty = self.get_target_point()
            tx_1, ty_1, tx_2, ty_2, tx_3, ty_3, tx_4, ty_4 = self.get_own_target(tx, ty)    # get each robot's target 
            rospy.loginfo("(tx: %f, ty: %f), (cx_1: %f, cy_1: %f)", tx, ty, self.robot_pose_1.position.x, self.robot_pose_1.position.y)
            
            # Drive to target points (using move2pose)
            quaternion_1 = (self.robot_pose_1.orientation.x, self.robot_pose_1.orientation.y, self.robot_pose_1.orientation.z, self.robot_pose_1.orientation.w)
            quaternion_2 = (self.robot_pose_2.orientation.x, self.robot_pose_2.orientation.y, self.robot_pose_2.orientation.z, self.robot_pose_2.orientation.w)
            quaternion_3 = (self.robot_pose_3.orientation.x, self.robot_pose_3.orientation.y, self.robot_pose_3.orientation.z, self.robot_pose_3.orientation.w)
            quaternion_4 = (self.robot_pose_4.orientation.x, self.robot_pose_4.orientation.y, self.robot_pose_4.orientation.z, self.robot_pose_4.orientation.w)
            euler_1 = tf.transformations.euler_from_quaternion(quaternion_1)
            euler_2 = tf.transformations.euler_from_quaternion(quaternion_2)
            euler_3 = tf.transformations.euler_from_quaternion(quaternion_3)
            euler_4 = tf.transformations.euler_from_quaternion(quaternion_4)
            target_theta_1 = math.atan2(ty-self.robot_pose_1.position.y, tx-self.robot_pose_1.position.x)
            target_theta_2 = math.atan2(ty-self.robot_pose_2.position.y, tx-self.robot_pose_2.position.x)
            target_theta_3 = math.atan2(ty-self.robot_pose_3.position.y, tx-self.robot_pose_3.position.x)
            target_theta_4 = math.atan2(ty-self.robot_pose_4.position.y, tx-self.robot_pose_4.position.x)
            v1, w1 = self.move2pose(tx_1, ty_1, target_theta_1, self.robot_pose_1.position.x, self.robot_pose_1.position.y, euler_1[2])
            v2, w2 = self.move2pose(tx_2, ty_2, target_theta_2, self.robot_pose_2.position.x, self.robot_pose_2.position.y, euler_2[2])
            v3, w3 = self.move2pose(tx_3, ty_3, target_theta_3, self.robot_pose_3.position.x, self.robot_pose_3.position.y, euler_3[2])
            v4, w4 = self.move2pose(tx_4, ty_4, target_theta_4, self.robot_pose_4.position.x, self.robot_pose_4.position.y, euler_4[2])

            if abs(v1) > MAX_LINEAR_SPEED:
                v1= np.sign(v1) * MAX_LINEAR_SPEED
            if abs(w1) > MAX_ANGULAR_SPEED:
                w1 = np.sign(w1) * MAX_ANGULAR_SPEED
            if abs(v2) > MAX_LINEAR_SPEED:
                v2= np.sign(v2) * MAX_LINEAR_SPEED
            if abs(w2) > MAX_ANGULAR_SPEED:
                w2= np.sign(w2) * MAX_ANGULAR_SPEED
            if abs(v3) > MAX_LINEAR_SPEED:
                v3= np.sign(v3) * MAX_LINEAR_SPEED
            if abs(w3) > MAX_ANGULAR_SPEED:
                w3 = np.sign(w3) * MAX_ANGULAR_SPEED
            if abs(v4) > MAX_LINEAR_SPEED:
                v4= np.sign(v4) * MAX_LINEAR_SPEED
            if abs(w4) > MAX_ANGULAR_SPEED:
                w4 = np.sign(w4) * MAX_ANGULAR_SPEED

            # Publish v, w
            twist_1.linear.x = v1
            twist_1.angular.z = w1
            self.velocity_publisher_1.publish(twist_1)
            twist_2.linear.x = v2
            twist_2.angular.z = w2
            self.velocity_publisher_2.publish(twist_2)
            twist_3.linear.x = v3
            twist_3.angular.z = w3
            self.velocity_publisher_3.publish(twist_3)
            twist_4.linear.x = v4
            twist_4.angular.z = w4
            self.velocity_publisher_4.publish(twist_4)

            # visualize
            # self.waypoint_rviz()
            self.rate.sleep()
    
    def move2pose(self, x_goal, y_goal, theta_goal, x, y, theta):
        """
        Constructs an instantiate of the PathFinderController for navigating a
        3-DOF wheeled robot on a 2D plane
        Parameters
        ----------
        Kp_rho : The linear velocity gain to translate the robot along a line
                towards the goal
        Kp_alpha : The angular velocity gain to rotate the robot towards the goal
        Kp_beta : The offset angular velocity gain accounting for smooth merging to
                the goal angle (i.e., it helps the robot heading to be parallel
                to the target angle.)
        Returns the control command for the linear and angular velocities as
            well as the distance to goal
            Parameters
            ----------
            x_diff : The position of target with respect to current robot position
                    in x direction
            y_diff : The position of target with respect to current robot position
                    in y direction
            theta : The current heading angle of robot with respect to x axis
            theta_goal: The target angle of robot with respect to x axis
            Returns
            -------
            rho : The distance between the robot and the goal position
            v : Command linear velocity
            w : Command angular velocity
            

            Description of local variables:
            - alpha is the angle to the goal relative to the heading of the robot
            - beta is the angle between the robot's position and the goal
            position plus the goal angle
            - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
            the goal
            - Kp_beta*beta rotates the line so that it is parallel to the goal
            angle
            
            Note:
            we restrict alpha and beta (angle differences) to the range
            [-pi, pi] to prevent unstable behavior e.g. difference going
            from 0 rad to 2*pi rad with slight turn
        """
        x_diff = x_goal - x
        y_diff = y_goal - y
        
        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        v = 0.0 if rho < 0.2 else self.Kp_rho * rho
        w = 0.0 if v == 0 else self.Kp_alpha * alpha + self.Kp_beta * beta

        if np.pi / 2 < alpha < np.pi or -np.pi < alpha < -np.pi / 2:
            v = -v     
        return v, w
    

if __name__ == '__main__':
    try:
        vs = VirtualStructure()
        vs.virtual_structure()
    except rospy.ROSInterruptException:
        pass