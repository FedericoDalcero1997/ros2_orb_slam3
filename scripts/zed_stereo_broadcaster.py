#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


class ZEDStereoTF(Node):

    def __init__(self):
        super().__init__('zed_stereo_tf')

        self.br = tf2_ros.StaticTransformBroadcaster(self)

        t = TransformStamped()

        # baseline RIGHT relative to LEFT
        t.header.frame_id = "zed_left_camera_frame"
        t.child_frame_id = "zed_right_camera_frame"

        # 🔴 QUI METTI IL TUO BASELINE REALE
        t.transform.translation.x = 0.10   # 10 cm ORIZZONTALE
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)
        self.get_logger().info("Static stereo baseline published")


def main():
    rclpy.init()
    node = ZEDStereoTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
