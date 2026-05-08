#!/usr/bin/env python3


"""
Python node for the MonocularMode cpp node.

Minimal adaptation for live ZED camera input:
- keeps the original handshake
- keeps the original topics
- keeps the original main loop structure as much as possible
- replaces dataset image loading with subscription to ZED image topic
- resizes images to 640x400 preserving the original 16:10 aspect ratio

Author: Azmyin Md. Kamal
Adapted for ZED live input
"""

# Imports
import sys
import os
import glob
import time
import copy
import shutil
from pathlib import Path
import argparse
import natsort
import yaml
import copy
import numpy as np
import cv2

# ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError


class MonoDriver(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)

        # Initialize parameters
        self.declare_parameter("settings_name", "ZEDXOneGS")
        self.declare_parameter("image_seq", "NULL")

        # Parse values sent by command line
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.image_seq = str(self.get_parameter('image_seq').value)

        # DEBUG
        print(f"-------------- Received parameters --------------------------\n")
        print(f"self.settings_name: {self.settings_name}")
        print(f"self.image_seq: {self.image_seq}")
        print()

        # Kept only to minimize changes from original file
        self.home_dir = str(Path.home()) + "/orbslamv3/ros2_test/src/ros2_orb_slam3"
        self.parent_dir = "TEST_DATASET"
        self.image_sequence_dir = self.home_dir + "/" + self.parent_dir + "/" + self.image_seq

        print(f"self.image_sequence_dir: {self.image_sequence_dir}\n")

        # Global variables
        self.node_name = "mono_py_driver"
        self.image_seq_dir = ""
        self.imgz_seqz = []
        self.time_seqz = []

        # Define a CvBridge object
        self.br = CvBridge()

        # Live ZED cache
        self.latest_frame = None
        self.latest_timestamp = None
        self.latest_header = None
        self.last_published_timestamp = None

        # ROS2 publisher/subscriber variables [HARDCODED]
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.send_config = True

        # Setup ROS2 publishers and subscribers
        self.publish_exp_config_ = self.create_publisher(
            String,
            self.pub_exp_config_name,
            10
        )

        # Build the configuration string to be sent out
        self.exp_config_msg = self.settings_name
        print(f"Configuration to be sent: {self.exp_config_msg}")

        # Subscriber to get acknowledgement from CPP node
        self.subscribe_exp_ack_ = self.create_subscription(
            String,
            self.sub_exp_ack_name,
            self.ack_callback,
            10
        )
        self.subscribe_exp_ack_

        # Publishers
        self.publish_img_msg_ = self.create_publisher(
            Image,
            self.pub_img_to_agent_name,
            10
        )

        self.publish_timestep_msg_ = self.create_publisher(
            Float64,
            self.pub_timestep_to_agent_name,
            10
        )

        # Subscribe to ZED grayscale image
        self.zed_sub_ = self.create_subscription(
            Image,
            '/zed/zed_node/gray/rect/image',
            self.zed_callback,
            qos_profile_sensor_data
        )

        # Initialize work variables for main logic
        self.start_frame = 0
        self.end_frame = -1
        self.frame_stop = -1
        self.show_imgz = False
        self.frame_id = 0
        self.frame_count = 0
        self.inference_time = []

        print()
        print("MonoDriver initialized, attempting handshake with CPP node")

    # ****************************************************************************************
    def zed_callback(self, msg):
        """
        Receive latest frame from ZED and cache it.
        """
        try:
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding="mono8")
            frame = cv2.resize(frame, (640, 400), interpolation=cv2.INTER_AREA)

            self.latest_frame = frame
            self.latest_timestamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            self.latest_header = msg.header
        except Exception as e:
            print(f"zed_callback error: {e}")
    # ****************************************************************************************

    # ****************************************************************************************
    def ack_callback(self, msg):
        """
        Callback function
        """
        print(f"Got ack: {msg.data}")

        if(msg.data == "ACK"):
            self.send_config = False
    # ****************************************************************************************

    # ****************************************************************************************
    def handshake_with_cpp_node(self):
        """
        Send and receive acknowledge of sent configuration settings
        """
        if self.send_config is True:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    # ****************************************************************************************

    # ****************************************************************************************
    def run_py_node(self, idx, imgz_name):
        """
        Master function that sends the image message to the CPP node.
        Minimal change: uses latest ZED frame instead of reading from disk.
        """

        if self.latest_frame is None or self.latest_timestamp is None:
            print("[PY] No frame yet, skipping publish")
            return

        if self.last_published_timestamp == self.latest_timestamp:
            print("[PY] Same frame as previous, skipping publish")
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self.latest_timestamp) > 0.5:
            print(f"[PY] Stale frame ({now - self.latest_timestamp:.3f}s old), skipping")
            return

        timestep = self.latest_timestamp
        self.frame_id = self.frame_id + 1

        img_msg = self.br.cv2_to_imgmsg(self.latest_frame, encoding="mono8")

        if self.latest_header is not None:
            img_msg.header = self.latest_header
            img_msg.header.stamp = self.latest_header.stamp

        timestep_msg = Float64()
        timestep_msg.data = timestep

        try:
            self.publish_timestep_msg_.publish(timestep_msg)
            
            self.publish_img_msg_.publish(img_msg)

            self.last_published_timestamp = self.latest_timestamp

        except CvBridgeError as e:
            print(e)
        except Exception as e:
            print(f"[PY] publish error: {e}")
    # ****************************************************************************************


# main function
def main(args=None):
    rclpy.init(args=args)
    n = MonoDriver("mono_py_node")

    # Blocking loop to initialize handshake
    while n.send_config is True:
        n.handshake_with_cpp_node()
        rclpy.spin_once(n, timeout_sec=0.1)

        if n.send_config is False:
            break

    print("Handshake complete")

    # Small delay after ACK
    time.sleep(1.0)

    # Blocking loop to send image and timestep message
    try:
        target_period = 1.0 / 10.0
        while rclpy.ok():
            start_time = time.time()
            rclpy.spin_once(n, timeout_sec=target_period * 0.8)
            n.run_py_node(0, "")
            elapsed = time.time() - start_time
            sleep_time = target_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass

    # Cleanup
    cv2.destroyAllWindows()
    n.destroy_node()
    rclpy.shutdown()


# Dunders, this .py is the main file
if __name__ == "__main__":
    main()
