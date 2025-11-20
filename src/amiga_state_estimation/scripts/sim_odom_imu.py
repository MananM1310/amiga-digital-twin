#!/usr/bin/env python3
import math
import random
import rospy
import tf

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def angle_wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SimOdomImuPublisher(object):
    def __init__(self):
        self.robot_model_name = rospy.get_param("~robot_model_name", "amiga_model")

        # Noise params (you can tune or turn off)
        self.odom_pos_std = rospy.get_param("~odom_pos_std", 0.01)      # m
        self.odom_yaw_std = rospy.get_param("~odom_yaw_std", 0.002)    # rad
        self.imu_angular_std = rospy.get_param("~imu_angular_std", 0.002)  # rad/s
        self.imu_linear_std = rospy.get_param("~imu_linear_std", 0.05)     # m/s^2

        self.odom_pub = rospy.Publisher("/sim/wheel_odom", Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher("/sim/imu", Imu, queue_size=10)

        # self.tf_broadcaster = tf.TransformBroadcaster()

        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None

        self.sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1
        )

        rospy.loginfo("SimOdomImuPublisher: using model_name=%s", self.robot_model_name)

    def model_states_cb(self, msg):
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            return

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        x_true = pose.position.x
        y_true = pose.position.y
        z_true = pose.position.z

        q = pose.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_true = math.atan2(siny_cosp, cosy_cosp)

        t = rospy.Time.now()

        # --- Fake wheel odometry (odom frame) ---
        # Add small noise
        x_odom = x_true + random.gauss(0.0, self.odom_pos_std)
        y_odom = y_true + random.gauss(0.0, self.odom_pos_std)
        yaw_odom = yaw_true + random.gauss(0.0, self.odom_yaw_std)

        quat_odom = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_odom)

        # Compute velocities from diff if we have previous data
        if self.prev_time is None:
            vx = 0.0
            vy = 0.0
            vyaw = 0.0
        else:
            dt = (t - self.prev_time).to_sec()
            if dt <= 0:
                vx = vy = vyaw = 0.0
            else:
                dx = x_odom - self.prev_x
                dy = y_odom - self.prev_y
                dyaw = angle_wrap(yaw_odom - self.prev_yaw)

                vx = (math.cos(yaw_odom) * dx + math.sin(yaw_odom) * dy) / dt
                vy = (-math.sin(yaw_odom) * dx + math.cos(yaw_odom) * dy) / dt
                vyaw = dyaw / dt

        self.prev_time = t
        self.prev_x = x_odom
        self.prev_y = y_odom
        self.prev_yaw = yaw_odom

        odom_msg = Odometry()
        odom_msg.header.stamp = t
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = x_odom
        odom_msg.pose.pose.position.y = y_odom
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quat_odom[0]
        odom_msg.pose.pose.orientation.y = quat_odom[1]
        odom_msg.pose.pose.orientation.z = quat_odom[2]
        odom_msg.pose.pose.orientation.w = quat_odom[3]

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vyaw

        # Simple covariances
        odom_msg.pose.covariance[0] = self.odom_pos_std ** 2
        odom_msg.pose.covariance[7] = self.odom_pos_std ** 2
        odom_msg.pose.covariance[35] = self.odom_yaw_std ** 2

        odom_msg.twist.covariance[0] = 0.1
        odom_msg.twist.covariance[7] = 0.1
        odom_msg.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom_msg)

        # Publish TF odom -> base_link
        # self.tf_broadcaster.sendTransform(
        #     (x_odom, y_odom, 0.0),
        #     quat_odom,
        #     t,
        #     "base_link",
        #     "odom",
        # )

        # --- Fake IMU ---
        imu_msg = Imu()
        imu_msg.header.stamp = t
        imu_msg.header.frame_id = "base_link"

        # Use true orientation (could also use noisy)
        quat_true = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_true)
        imu_msg.orientation.x = quat_true[0]
        imu_msg.orientation.y = quat_true[1]
        imu_msg.orientation.z = quat_true[2]
        imu_msg.orientation.w = quat_true[3]

        # Angular velocity from twist + noise
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = twist.angular.z + random.gauss(0.0, self.imu_angular_std)

        # Linear acceleration: here we just approximate from twist (or set zero)
        imu_msg.linear_acceleration.x = random.gauss(0.0, self.imu_linear_std)
        imu_msg.linear_acceleration.y = random.gauss(0.0, self.imu_linear_std)
        imu_msg.linear_acceleration.z = 9.81 + random.gauss(0.0, self.imu_linear_std)

        # Covariances (rough)
        imu_msg.orientation_covariance[0] = (self.odom_yaw_std ** 2)
        imu_msg.orientation_covariance[4] = (self.odom_yaw_std ** 2)
        imu_msg.orientation_covariance[8] = (self.odom_yaw_std ** 2)

        imu_msg.angular_velocity_covariance[8] = self.imu_angular_std ** 2
        imu_msg.linear_acceleration_covariance[0] = self.imu_linear_std ** 2
        imu_msg.linear_acceleration_covariance[4] = self.imu_linear_std ** 2
        imu_msg.linear_acceleration_covariance[8] = self.imu_linear_std ** 2

        self.imu_pub.publish(imu_msg)


if __name__ == "__main__":
    rospy.init_node("sim_odom_imu_publisher")
    node = SimOdomImuPublisher()
    rospy.spin()
