#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from gazebo_msgs.msg import ModelStates


class RowGoStraight(object):
    def __init__(self):
        # Publishers
        self.cmd_pub = rospy.Publisher("/amiga/cmd_vel", TwistStamped, queue_size=10)

        # Params
        self.forward_speed = rospy.get_param("~forward_speed", 0.4)
        self.rate_hz = rospy.get_param("~rate_hz", 20.0)

        # Stop logic
        self.stop_distance = rospy.get_param("~stop_distance", 0.5)   # meters
        self.stop_duration = rospy.get_param("~stop_duration", 3.0)  # seconds

        self.robot_model_name = rospy.get_param("~robot_model_name", "amiga_model")

        # State machine
        self.state = "DRIVE"
        self.state_start_time = rospy.Time.now()

        # Latest poses
        self.robot_x = None
        self.robot_y = None
        self.weed_x = None
        self.weed_y = None

        # Subscribers
        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1
        )
        self.nearest_weed_sub = rospy.Subscriber(
            "/sim/nearest_weed", PoseStamped, self.nearest_weed_cb, queue_size=1
        )

        rospy.loginfo(
            "row_go_straight: weed-based stopping enabled. "
            "forward_speed=%.2f, stop_distance=%.2f, stop_duration=%.1f",
            self.forward_speed, self.stop_distance, self.stop_duration
        )

    def model_states_cb(self, msg):
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            return
        pose = msg.pose[idx]
        self.robot_x = pose.position.x
        self.robot_y = pose.position.y

    def nearest_weed_cb(self, msg):
        self.weed_x = msg.pose.position.x
        self.weed_y = msg.pose.position.y

    def publish_cmd(self, linear_x, angular_z=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def weed_distance(self):
        if self.robot_x is None or self.robot_y is None:
            return None
        if self.weed_x is None or self.weed_y is None:
            return None
        dx = self.weed_x - self.robot_x
        dy = self.weed_y - self.robot_y
        return math.sqrt(dx*dx + dy*dy)

    def run(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            elapsed = (now - self.state_start_time).to_sec()

            if self.state == "DRIVE":
                self.publish_cmd(self.forward_speed)

                d = self.weed_distance()
                if d is not None:
                    rospy.loginfo_throttle(
                        1.0,
                        "row_go_straight: robot=(%.2f, %.2f), weed=(%.2f, %.2f), d=%.2f",
                        self.robot_x, self.robot_y,
                        self.weed_x, self.weed_y,
                        d
                    )
                    if d <= self.stop_distance:
                        rospy.loginfo(
                            "row_go_straight: d=%.2f <= %.2f, switching to STOP",
                            d, self.stop_distance
                        )
                        self.state = "STOP"
                        self.state_start_time = now

            elif self.state == "STOP":
                self.publish_cmd(0.0)
                if elapsed >= self.stop_duration:
                    rospy.loginfo("row_go_straight: finished wait, switching to DRIVE")
                    self.state = "DRIVE"
                    self.state_start_time = now

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("row_go_straight")
    node = RowGoStraight()
    node.run()
