#!/usr/bin/env python3
"""
Waypoint Navigator for AMIGA Robot

Simple waypoint navigation controller that drives the robot to (x,y) goal coordinates.
Uses a bi-directional control strategy:
1. Calculate bearing to goal
2. If goal is behind (>90 deg), drive backwards
3. Turn while driving (proportional heading control)

Author: AMIGA Digital Twin Project
"""

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
from std_msgs.msg import String


def angle_wrap(angle):
    """Wrap angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointNavigator:
    def __init__(self):
        rospy.loginfo("Initializing Waypoint Navigator (Bi-directional)...")
        
        # Parameters
        self.linear_speed = rospy.get_param("~linear_speed", 0.8)
        self.angular_speed = rospy.get_param("~angular_speed", 1.5)  # Increased from 0.5
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.3)
        self.heading_tolerance = rospy.get_param("~heading_tolerance", math.radians(5))
        self.kp_heading = rospy.get_param("~kp_heading", 2.0)  # Increased from 0.5
        self.invert_turn = rospy.get_param("~invert_turn", False)
        # Reduce speed as we approach the goal to avoid oscillation
        self.slow_down_radius = rospy.get_param("~slow_down_radius", 0.4)
        
        # Robot state
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        
        # Goal state
        self.goal_x = None
        self.goal_y = None
        self.has_goal = False
        
        # Navigation state
        self.state = "IDLE"  # IDLE, ROTATING, DRIVING, REACHED
        
        # Publishers
        self.cmd_pub = rospy.Publisher("/amiga/cmd_vel", TwistStamped, queue_size=10)
        self.status_pub = rospy.Publisher("/waypoint_nav/status", String, queue_size=10)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber("/sim/wheel_odom", Odometry, self.odom_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber("/waypoint_nav/goal", PoseStamped, self.goal_callback, queue_size=10)
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_callback, queue_size=1)
        
        rospy.loginfo("Waypoint Navigator initialized. Waiting for goals on /waypoint_nav/goal")
        rospy.loginfo(f"Parameters: linear_speed={self.linear_speed}, angular_speed={self.angular_speed}")
        rospy.loginfo(f"            goal_tolerance={self.goal_tolerance}")

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg):
        """Receive new goal waypoint"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.has_goal = True
        self.state = "NAVIGATING"
        
        rospy.loginfo(f"New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def clicked_point_callback(self, msg):
        """Receive a clicked point from RViz (/clicked_point) and set as goal"""
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.has_goal = True
        self.state = "NAVIGATING"

        rospy.loginfo(
            f"New goal from click in frame '{msg.header.frame_id or 'world'}': "
            f"({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def compute_distance_and_bearing(self):
        """Calculate distance and bearing to goal"""
        if self.robot_x is None or not self.has_goal:
            return None, None
        
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        
        return distance, bearing

    def publish_cmd(self, linear_x, angular_z):
        """Publish velocity command"""
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def navigate(self):
        """Main navigation control loop"""
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown():
            # Publish status
            self.status_pub.publish(self.state)
            
            # Wait for robot pose
            if self.robot_x is None:
                rospy.loginfo_throttle(2.0, "Waiting for odometry...")
                rate.sleep()
                continue
            
            # If no goal, stay idle
            if not self.has_goal:
                self.publish_cmd(0.0, 0.0)
                self.state = "IDLE"
                rate.sleep()
                continue
            
            # Compute distance and bearing to goal
            distance, bearing = self.compute_distance_and_bearing()

            if distance is None:
                rate.sleep()
                continue

            # Check if goal reached
            if distance <= self.goal_tolerance:
                self.publish_cmd(0.0, 0.0)
                if self.state != "REACHED":
                    rospy.loginfo(f"Goal reached! Final distance: {distance:.3f}m")
                self.state = "REACHED"
                self.has_goal = False  # Clear goal
                rate.sleep()
                continue

            # Calculate heading error
            heading_error = angle_wrap(bearing - self.robot_yaw)

            linear_vel = 0.0
            angular_vel = 0.0

            # If heading error is large, rotate in place to face the goal
            if abs(heading_error) > self.heading_tolerance:
                angular_vel = max(-self.angular_speed,
                                  min(self.angular_speed, self.kp_heading * heading_error))
                if self.invert_turn:
                    angular_vel = -angular_vel
                self.publish_cmd(0.0, angular_vel)
                if self.state != "ROTATING":
                    rospy.loginfo(f"Rotating to align. Heading error: {math.degrees(heading_error):.1f}°")
                self.state = "ROTATING"
            else:
                # Drive forward with heading correction
                angular_vel = max(-self.angular_speed,
                                  min(self.angular_speed, self.kp_heading * heading_error))
                if self.invert_turn:
                    angular_vel = -angular_vel

                if distance < self.slow_down_radius:
                    linear_vel = self.linear_speed * max(0.2, distance / self.slow_down_radius)
                else:
                    linear_vel = self.linear_speed

                self.publish_cmd(linear_vel, angular_vel)
                self.state = "DRIVING"

            rospy.loginfo_throttle(
                0.5,
                f"{self.state}: dist={distance:.2f}m, err={math.degrees(heading_error):.1f}°, v={linear_vel:.2f}, w={angular_vel:.2f}"
            )

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("waypoint_navigator")
    navigator = WaypointNavigator()
    navigator.navigate()
