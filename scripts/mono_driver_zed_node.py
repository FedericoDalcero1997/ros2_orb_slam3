#!/usr/bin/env python3

import time
import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge


class MonoDriverZED(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)

        self.declare_parameter("settings_name", "ZEDXOneGS")
        self.settings_name = str(self.get_parameter("settings_name").value)

        print("-------------- Received parameters --------------------------\n")
        print(f"self.settings_name: {self.settings_name}")
        print()

        self.br = CvBridge()

        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"

        self.send_config = True
        self.handshake_done = False
        self.ready_to_stream = False

        self.latest_img_msg = None
        self.latest_timestamp = None

        self.publish_exp_config_ = self.create_publisher(
            String,
            self.pub_exp_config_name,
            1
        )

        self.exp_config_msg = self.settings_name
        print(f"Configuration to be sent: {self.exp_config_msg}")

        self.subscribe_exp_ack_ = self.create_subscription(
            String,
            self.sub_exp_ack_name,
            self.ack_callback,
            10
        )

        self.publish_img_msg_ = self.create_publisher(
            Image,
            self.pub_img_to_agent_name,
            1
        )

        self.publish_timestep_msg_ = self.create_publisher(
            Float64,
            self.pub_timestep_to_agent_name,
            1
        )

        self.zed_image_sub_ = self.create_subscription(
            Image,
            "/zed/zed_node/rgb/rect/image",
            self.zed_callback,
            qos_profile_sensor_data
        )

        # timer separato per pubblicare in modo ordinato
        self.publish_timer_ = self.create_timer(1.0 / 20.0, self.publish_latest_frame)

        print()
        print("MonoDriverZED initialized, attempting handshake with CPP node")

    def ack_callback(self, msg):
        print(f"Got ack: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False
            self.handshake_done = True

            # importantissimo: ACK arriva prima di initializeVSLAM nel C++
            # quindi aspettiamo un attimo prima di iniziare lo streaming
            time.sleep(1.0)
            self.ready_to_stream = True

    def handshake_with_cpp_node(self):
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    def zed_callback(self, msg):
        if not self.handshake_done:
            return

        try:
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img_msg = self.br.cv2_to_imgmsg(frame, encoding="passthrough")
            img_msg.header = msg.header

            # salva solo l'ultimo frame disponibile
            self.latest_img_msg = img_msg
            self.latest_timestamp = float(msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)

        except Exception as e:
            self.get_logger().error(f"Error in zed_callback: {e}")

    def publish_latest_frame(self):
        if not self.ready_to_stream:
            return

        if self.latest_img_msg is None or self.latest_timestamp is None:
            return

        try:
            timestep_msg = Float64()
            timestep_msg.data = self.latest_timestamp

            # copia difensiva
            img_msg = copy.deepcopy(self.latest_img_msg)

            # ordine identico al driver originale
            self.publish_timestep_msg_.publish(timestep_msg)
            self.publish_img_msg_.publish(img_msg)

            self.get_logger().info(
                f"Published frame ns={int(timestep_msg.data)}",
                throttle_duration_sec=2.0
            )

        except Exception as e:
            self.get_logger().error(f"Error in publish_latest_frame: {e}")


def main(args=None):
    rclpy.init(args=args)
    n = MonoDriverZED("mono_py_node")

    while n.send_config:
        n.handshake_with_cpp_node()
        rclpy.spin_once(n, timeout_sec=0.1)
        if not n.send_config:
            break

    print("Handshake complete")

    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
