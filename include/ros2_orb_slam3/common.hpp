// Include file 
#ifndef COMMON_HPP
#define COMMON_HPP

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <deque>
#include <cstring>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1;

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.h>

#include "System.h"

#define pass (void)0

class MonocularMode : public rclcpp::Node
{
public:
    std::string experimentConfig = "";
    double timeStep;
    std::string receivedConfig = "";

    MonocularMode();
    ~MonocularMode();

private:
    std::string homeDir = "";
    std::string packagePath = "orbslamv3/ros2_test/src/ros2_orb_slam3/";
    std::string nodeName = "";
    std::string vocFilePath = "";
    std::string settingsFilePath = "";
    bool bSettingsFromPython = false;

    std::string subexperimentconfigName = "";
    std::string pubconfigackName = "";
    std::string subImgMsgName = "";
    std::string subTimestepMsgName = "";

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   expConfig_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      configAck_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr  subTimestepMsg_subscription_;

    ORB_SLAM3::System* pAgent = nullptr;
    ORB_SLAM3::System::eSensor sensorType;
    bool enablePangolinWindow = true;
    bool enableOpenCVWindow   = true;

    void experimentSetting_callback(const std_msgs::msg::String& msg);
    void Timestep_callback(const std_msgs::msg::Float64& time_msg);
    void Img_callback(const sensor_msgs::msg::Image& msg);
    void initializeVSLAM(std::string& configString);
};

#endif
