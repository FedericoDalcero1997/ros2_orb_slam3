#!/usr/bin/env python3

import time
import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64

from message_filters import Subscriber, ApproximateTimeSynchronizer

from cv_bridge import CvBridge


class StereoDriverZED(Node):

    def __init__(self):

        super().__init__("stereo_py_node")

        self.declare_parameter("settings_name", "ZEDXOneGS")
        self.settings_name = str(
            self.get_parameter("settings_name").value
        )

        self.br = CvBridge()

        # handshake
        self.send_config = True
        self.handshake_done = False
        self.ready_to_stream = False

        # latest synced frames
        self.latest_left = None
        self.latest_right = None
        self.latest_timestamp = None

        # publishers
        self.pub_exp_config = self.create_publisher(
            String,
            "/stereo_py_driver/experiment_settings",
            1
        )

        self.pub_left = self.create_publisher(
            Image,
            "/stereo_py_driver/left_image",
            1
        )

        self.pub_right = self.create_publisher(
            Image,
            "/stereo_py_driver/right_image",
            1
        )

        self.pub_timestamp = self.create_publisher(
            Float64,
            "/stereo_py_driver/timestep_msg",
            1
        )

        # ack subscriber
        self.sub_ack = self.create_subscription(
            String,
            "/stereo_py_driver/exp_settings_ack",
            self.ack_callback,
            10
        )

        # stereo subscribers
        self.left_sub = Subscriber(
            self,
            Image,
            "/zed_left/zed_node/gray/rect/image",
            qos_profile=qos_profile_sensor_data
        )

        self.right_sub = Subscriber(
            self,
            Image,
            "/zed_right/zed_node/gray/rect/image",
            qos_profile=qos_profile_sensor_data
        )

        # sync
        self.ts = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=10,
            slop=0.02
        )

        self.ts.registerCallback(self.stereo_callback)

        # publish timer
        self.publish_timer = self.create_timer(
            1.0 / 20.0,
            self.publish_latest_stereo
        )

        self.get_logger().info("StereoDriverZED initialized")

    def ack_callback(self, msg):

        if msg.data == "ACK":

            self.send_config = False
            self.handshake_done = True

            time.sleep(1.0)

            self.ready_to_stream = True

    def handshake_with_cpp_node(self):

        if self.send_config:

            msg = String()
            msg.data = self.settings_name

            self.pub_exp_config.publish(msg)

    def stereo_callback(self, left_msg, right_msg):

        if not self.handshake_done:
            return

        try:

            left_frame = self.br.imgmsg_to_cv2(
                left_msg,
                desired_encoding="mono8"
            )

            right_frame = self.br.imgmsg_to_cv2(
                right_msg,
                desired_encoding="mono8"
            )

            left_ros = self.br.cv2_to_imgmsg(
                left_frame,
                encoding="mono8"
            )

            right_ros = self.br.cv2_to_imgmsg(
                right_frame,
                encoding="mono8"
            )

            left_ros.header = left_msg.header
            right_ros.header = right_msg.header

            self.latest_left = left_ros
            self.latest_right = right_ros

            self.latest_timestamp = (
                left_msg.header.stamp.sec * 1e9
                + left_msg.header.stamp.nanosec
            )

        except Exception as e:

            self.get_logger().error(
                f"Stereo callback error: {e}"
            )

    def publish_latest_stereo(self):

        if not self.ready_to_stream:
            return

        if self.latest_left is None:
            return

        try:

            ts_msg = Float64()
            ts_msg.data = float(self.latest_timestamp)

            left = copy.deepcopy(self.latest_left)
            right = copy.deepcopy(self.latest_right)

            self.pub_timestamp.publish(ts_msg)

            self.pub_left.publish(left)
            self.pub_right.publish(right)

        except Exception as e:

            self.get_logger().error(
                f"Publish stereo error: {e}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = StereoDriverZED()

    while node.send_config:

        node.handshake_with_cpp_node()

        rclpy.spin_once(node, timeout_sec=0.1)

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()
