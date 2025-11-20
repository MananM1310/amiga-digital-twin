#!/usr/bin/env python3
import math
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates  # <-- added this
from cv_bridge import CvBridge, CvBridgeError


class RowFollowCamera(object):
    def __init__(self):
        self.bridge = CvBridge()

        # --- Topics ---
        self.image_topic = rospy.get_param("~image_topic", "/oak_d_camera_1/image_raw")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/amiga/cmd_vel")

        # --- Motion / control params ---
        self.forward_speed = rospy.get_param("~forward_speed", 0.4)
        self.kp_row = rospy.get_param("~kp_row", 0.6)
        self.invert_steering = rospy.get_param("~invert_steering", False)

        # --- Lane / vision params ---
        self.num_scan_lines = rospy.get_param("~num_scan_lines", 20)
        self.min_line_points = rospy.get_param("~min_line_points", 5)

        self.lower_green = np.array(rospy.get_param("~lower_green", [35, 40, 40]))
        self.upper_green = np.array(rospy.get_param("~upper_green", [85, 255, 255]))

        self.lane_width_px = rospy.get_param("~lane_width_px", 70)

        # --- Weed stopping params ---
        self.use_weed_stopping = rospy.get_param("~use_weed_stopping", True)
        self.stop_distance = rospy.get_param("~stop_distance", 0.5)
        self.stop_duration = rospy.get_param("~stop_duration", 3.0)
        self.weed_min_separation = rospy.get_param("~weed_min_separation", 0.8)

        # --- Debug visualization ---
        self.publish_debug = rospy.get_param("~publish_debug", True)
        self.debug_image_topic = rospy.get_param(
            "~debug_image_topic", "/row_follow/debug_image"
        )

        # --- Internal state ---
        self.last_ang_z = 0.0

        # For weed logic
        self.robot_x = None
        self.robot_y = None
        self.nearest_weed = None  # (wx, wy)
        self.visited_weeds = []   # list of (wx, wy)
        self.state = "DRIVE"
        self.stop_end_time = rospy.Time(0)

        # --- Publishers / Subscribers ---
        self.cmd_pub = rospy.Publisher(self.cmd_topic, TwistStamped, queue_size=10)

        if self.publish_debug:
            self.debug_pub = rospy.Publisher(
                self.debug_image_topic, Image, queue_size=1
            )
        else:
            self.debug_pub = None

        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_cb, queue_size=1
        )

        if self.use_weed_stopping:
            self.robot_x = None
            self.robot_y = None

            self.model_states_sub = rospy.Subscriber(
                "/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=10
            )
            self.weed_sub = rospy.Subscriber(
                "/sim/nearest_weed", PoseStamped, self.weed_cb, queue_size=10
            )
            self.robot_model_name = rospy.get_param("~robot_model_name", "amiga_model")

        rospy.loginfo(
            "row_follow_camera: lane following + weed stopping. image=%s, cmd=%s",
            self.image_topic,
            self.cmd_topic,
        )

    # -------------------------------------------------------------------------
    # ROS callbacks
    # -------------------------------------------------------------------------

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def weed_cb(self, msg):
        self.nearest_weed = (msg.pose.position.x, msg.pose.position.y)

    def image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn("row_follow_camera: cv_bridge error: %s", e)
            return

        # Rotate 90 deg clockwise so ground is at bottom
        img_rot = cv2.rotate(cv_img, cv2.ROTATE_90_CLOCKWISE)
        h, w, _ = img_rot.shape

        # --- 1) Threshold green over whole image ---
        hsv = cv2.cvtColor(img_rot, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # --- 2) Sample plant centers on scanlines ---
        xs = []
        ys_local = []

        scan_rows = np.linspace(h - 1, 0, num=self.num_scan_lines, dtype=int)
        for r in scan_rows:
            row_mask = mask[r, :]
            cols = np.where(row_mask > 0)[0]
            if cols.size == 0:
                continue
            mean_x = float(cols.mean())
            y_local = float(h - 1 - r)  # 0 at bottom
            xs.append(mean_x)
            ys_local.append(y_local)

        if len(xs) >= self.min_line_points:
            xs_arr = np.array(xs)
            ys_arr = np.array(ys_local)

            try:
                a, b = np.polyfit(ys_arr, xs_arr, 1)  # x = a*y + b
            except Exception as e:
                rospy.logwarn("row_follow_camera: polyfit failed: %s", e)
                self.publish_control_with_weed(self.forward_speed, self.last_ang_z)
                self.publish_debug_image(img_rot, mask, xs_arr, ys_arr, None)
                return

            # row center at bottom
            y_local_bottom = 0.0
            x_row_bottom = a * y_local_bottom + b
            center_x = w / 2.0
            error_px = x_row_bottom - center_x
            error_norm = error_px / center_x

            ang_z = -self.kp_row * error_norm
            if self.invert_steering:
                ang_z = -ang_z

            self.last_ang_z = ang_z

            rospy.loginfo_throttle(
                0.5,
                "row_follow_camera: %d pts, x_bottom=%.2f, center=%.2f, err_norm=%.3f, ang_z=%.3f",
                len(xs),
                x_row_bottom,
                center_x,
                error_norm,
                ang_z,
            )

            self.publish_control_with_weed(self.forward_speed, ang_z)
            self.publish_debug_image(img_rot, mask, xs_arr, ys_arr, (a, b))
        else:
            rospy.loginfo_throttle(
                1.0,
                "row_follow_camera: weak vision (%d pts). Keeping last_ang_z=%.3f",
                len(xs),
                self.last_ang_z,
            )
            self.publish_control_with_weed(self.forward_speed, self.last_ang_z)
            self.publish_debug_image(img_rot, mask, None, None, None)
    
    def model_states_cb(self, msg):
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            return

        pose = msg.pose[idx]
        self.robot_x = pose.position.x
        self.robot_y = pose.position.y

    # -------------------------------------------------------------------------
    # Weed stopping + cmd publish
    # -------------------------------------------------------------------------

    def publish_control_with_weed(self, desired_linear_x, desired_ang_z):
        linear_x = desired_linear_x
        ang_z = desired_ang_z

        # If weed logic is disabled, just publish command and return
        if not self.use_weed_stopping:
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.x = linear_x
            cmd.twist.angular.z = ang_z
            self.cmd_pub.publish(cmd)
            return

        # Debug what we *have* before doing any math
        rospy.loginfo_throttle(
            0.5,
            "weed_debug_pre: state=%s, robot=(%s, %s), nearest_weed=%s",
            self.state,
            "None" if self.robot_x is None else f"{self.robot_x:.2f}",
            "None" if self.robot_y is None else f"{self.robot_y:.2f}",
            "None" if self.nearest_weed is None
            else f"({self.nearest_weed[0]:.2f}, {self.nearest_weed[1]:.2f})",
        )

        if self.robot_x is not None and self.nearest_weed is not None:
            wx, wy = self.nearest_weed
            dx = wx - self.robot_x
            dy = wy - self.robot_y
            d = math.hypot(dx, dy)

            already_visited = any(
                math.hypot(wx - vx, wy - vy) < self.weed_min_separation
                for (vx, vy) in self.visited_weeds
            )

            now = rospy.Time.now()

            rospy.loginfo_throttle(
                0.3,
                "weed_debug: state=%s robot=(%.2f, %.2f) weed=(%.2f, %.2f) "
                "d=%.3f stop_dist=%.3f visited=%s",
                self.state,
                self.robot_x,
                self.robot_y,
                wx,
                wy,
                d,
                self.stop_distance,
                already_visited,
            )

            if self.state == "DRIVE":
                if (d <= self.stop_distance) and (not already_visited):
                    rospy.loginfo(
                        "weed_debug: ENTER STOPPING d=%.3f <= %.3f (new weed).",
                        d,
                        self.stop_distance,
                    )
                    self.state = "STOPPING"
                    self.stop_end_time = now + rospy.Duration(self.stop_duration)
                    self.visited_weeds.append((wx, wy))
                    linear_x = 0.0

            elif self.state == "STOPPING":
                if now < self.stop_end_time:
                    linear_x = 0.0
                else:
                    rospy.loginfo("weed_debug: stop finished, back to DRIVE.")
                    self.state = "DRIVE"

        # If robot_x or nearest_weed are missing
        else:
            if self.robot_x is None:
                rospy.loginfo_throttle(1.0, "weed_debug: no wheel odom yet (/sim/wheel_odom).")
            if self.nearest_weed is None:
                rospy.loginfo_throttle(1.0, "weed_debug: no weed oracle yet (/sim/nearest_weed).")

        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = ang_z
        self.cmd_pub.publish(cmd)

    # -------------------------------------------------------------------------
    # Debug visualization: lane band
    # -------------------------------------------------------------------------

    def publish_debug_image(self, img_rot, mask, xs_arr, ys_arr, line_params):
        if not self.publish_debug or self.debug_pub is None:
            return

        h, w = img_rot.shape[:2]
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        overlay = cv2.addWeighted(img_rot, 0.7, mask_colored, 0.3, 0)

        # Red points
        if xs_arr is not None and ys_arr is not None:
            for x, y_local in zip(xs_arr, ys_arr):
                r = int(h - 1 - y_local)
                c = int(x)
                if 0 <= r < h and 0 <= c < w:
                    cv2.circle(overlay, (c, r), 3, (0, 0, 255), -1)

        # Lane band
        if line_params is not None:
            a, b = line_params

            num_samples = 12
            y_pix_samples = np.linspace(h - 1, int(h * 0.1), num=num_samples)

            centers = []
            for y_pix in y_pix_samples:
                y_pix_i = float(y_pix)
                y_local = float(h - 1 - y_pix_i)
                x_center = a * y_local + b
                centers.append(np.array([x_center, y_pix_i], dtype=np.float32))

            if len(centers) >= 2:
                p_bottom = centers[0]
                p_top = centers[-1]
                v = p_top - p_bottom
                norm_v = np.linalg.norm(v) + 1e-6
                v_dir = v / norm_v
                n_dir = np.array([-v_dir[1], v_dir[0]], dtype=np.float32)
                half_w = float(self.lane_width_px) / 2.0

                left_pts = []
                right_pts = []
                for p in centers:
                    offset = n_dir * half_w
                    p_left = p - offset
                    p_right = p + offset
                    left_pts.append(p_left)
                    right_pts.append(p_right)

                left_pts = np.array(left_pts, dtype=np.int32)
                right_pts = np.array(right_pts[::-1], dtype=np.int32)
                lane_pts = np.vstack([left_pts, right_pts])

                lane_pts[:, 0] = np.clip(lane_pts[:, 0], 0, w - 1)
                lane_pts[:, 1] = np.clip(lane_pts[:, 1], 0, h - 1)

                lane_mask = overlay.copy()
                cv2.fillConvexPoly(lane_mask, lane_pts, (0, 255, 255))
                overlay = cv2.addWeighted(overlay, 0.6, lane_mask, 0.4, 0)

                for i in range(len(left_pts) - 1):
                    cv2.line(overlay, tuple(left_pts[i]),
                             tuple(left_pts[i + 1]), (0, 0, 0), 2)
                for i in range(len(right_pts) - 1):
                    cv2.line(overlay, tuple(right_pts[i]),
                             tuple(right_pts[i + 1]), (0, 0, 0), 2)

        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            dbg_msg.header.stamp = rospy.Time.now()
            self.debug_pub.publish(dbg_msg)
        except CvBridgeError as e:
            rospy.logwarn("row_follow_camera: debug publish error: %s", e)


if __name__ == "__main__":
    rospy.init_node("row_follow_camera")
    node = RowFollowCamera()
    rospy.spin()
