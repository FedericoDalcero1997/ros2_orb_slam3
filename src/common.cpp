#include "ros2_orb_slam3/common.hpp"

MonocularMode::MonocularMode() : Node("mono_node_cpp")
{
    homeDir = getenv("HOME");

    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given");
    this->declare_parameter("voc_file_arg", "file_not_set");
    this->declare_parameter("settings_file_path_arg", "file_path_not_set");

    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();

    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    subexperimentconfigName = "/mono_py_driver/experiment_settings";
    pubconfigackName        = "/mono_py_driver/exp_settings_ack";
    subImgMsgName           = "/mono_py_driver/img_msg";
    subTimestepMsgName      = "/mono_py_driver/timestep_msg";

    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
        subexperimentconfigName, 10,
        std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(
        pubconfigackName, 10);

    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName, 10,
        std::bind(&MonocularMode::Img_callback, this, _1));

    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        subTimestepMsgName, 10,
        std::bind(&MonocularMode::Timestep_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

MonocularMode::~MonocularMode()
{
    RCLCPP_INFO(this->get_logger(), "Destructor called");
    if (pAgent)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down pAgent");
        pAgent->Shutdown();
    }
    pass;
}

void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg)
{
    static bool already_initialized = false;

    RCLCPP_INFO(this->get_logger(), "Received config string: %s", msg.data.c_str());

    if (already_initialized)
    {
        RCLCPP_WARN(this->get_logger(), "Ignoring duplicate experiment config");
        return;
    }

    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    receivedConfig   = experimentConfig;

    initializeVSLAM(experimentConfig);

    auto message = std_msgs::msg::String();
    message.data = "ACK";
    configAck_publisher_->publish(message);

    already_initialized = true;
}

void MonocularMode::initializeVSLAM(std::string& configString)
{
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");
        rclcpp::shutdown();
    }

    std::string fullSettingsFilePath = settingsFilePath + configString + ".yaml";
    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", fullSettingsFilePath.c_str());

    sensorType = ORB_SLAM3::System::MONOCULAR;  // <-- era IMU_MONOCULAR

    pAgent = new ORB_SLAM3::System(vocFilePath, fullSettingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "ORB_SLAM3::System created, pAgent=%p", (void*)pAgent);
}

void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg)
{
    timeStep = time_msg.data;
}

void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    if (!pAgent)
    {
        RCLCPP_ERROR(this->get_logger(), "pAgent is null, skipping TrackMonocular");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        return;
    }

    double imageTime =
        static_cast<double>(msg.header.stamp.sec) +
        static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, imageTime);

    if (Tcw.matrix().hasNaN())
    {
        RCLCPP_WARN(this->get_logger(), "Invalid pose, skipping");
        return;
    }

    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f pos = Twc.translation();
    RCLCPP_INFO(this->get_logger(),
        "Camera position: x=%.3f y=%.3f z=%.3f | dist=%.3f",
        pos.x(), pos.y(), pos.z(), pos.norm());
}
