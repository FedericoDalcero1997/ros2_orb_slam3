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
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Stereo/";
    }

    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    subexperimentconfigName = "/stereo_py_driver/experiment_settings";
    pubconfigackName        = "/stereo_py_driver/exp_settings_ack";
    subLeftImgMsgName       = "/stereo_py_driver/left_image";
    subRightImgMsgName      = "/stereo_py_driver/right_image";
    subTimestepMsgName      = "/stereo_py_driver/timestep_msg";

    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
        subexperimentconfigName, 10,
        std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(
        pubconfigackName, 10);

    subLeftImg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subLeftImgMsgName, 10,
        std::bind(&MonocularMode::LeftImg_callback, this, _1));

    subRightImg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subRightImgMsgName, 10,
        std::bind(&MonocularMode::RightImg_callback, this, _1));

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

    sensorType = ORB_SLAM3::System::STEREO;

    pAgent = new ORB_SLAM3::System(vocFilePath, fullSettingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "ORB_SLAM3::System created, pAgent=%p", (void*)pAgent);
}

void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg)
{
    timeStep = time_msg.data;
}

// void MonocularMode::Stereo_callback(
//     const sensor_msgs::msg::Image::ConstSharedPtr left_msg,
//     const sensor_msgs::msg::Image::ConstSharedPtr right_msg)
// {
//     if (!pAgent)
//     {
//         RCLCPP_ERROR(this->get_logger(), "pAgent null");
//         return;
//     }

//     cv_bridge::CvImagePtr cv_left;
//     cv_bridge::CvImagePtr cv_right;

//     try
//     {
//         cv_left = cv_bridge::toCvCopy(left_msg);
//         cv_right = cv_bridge::toCvCopy(right_msg);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         RCLCPP_ERROR(this->get_logger(), "cv_bridge error");
//         return;
//     }

//     double imageTime =
//         static_cast<double>(left_msg->header.stamp.sec) +
//         static_cast<double>(left_msg->header.stamp.nanosec) * 1e-9;

//     Sophus::SE3f Tcw =
//         pAgent->TrackStereo(
//             cv_left->image,
//             cv_right->image,
//             imageTime
//         );
// }

void MonocularMode::RightImg_callback(
    const sensor_msgs::msg::Image& msg)
{
    latestRightMsg = msg;
    hasRight = true;
}

void MonocularMode::LeftImg_callback(
    const sensor_msgs::msg::Image& msg)
{
    if (!pAgent)
    {
        RCLCPP_ERROR(this->get_logger(), "pAgent null");
        return;
    }

    if (!hasRight)
    {
        return;
    }

    latestLeftMsg = msg;
    hasLeft = true;

    cv_bridge::CvImagePtr cv_left;
    cv_bridge::CvImagePtr cv_right;

    try
    {
        cv_left = cv_bridge::toCvCopy(latestLeftMsg);
        cv_right = cv_bridge::toCvCopy(latestRightMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error");
        return;
    }

    double imageTime =
        static_cast<double>(msg.header.stamp.sec) +
        static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    Sophus::SE3f Tcw =
        pAgent->TrackStereo(
            cv_left->image,
            cv_right->image,
            imageTime
        );

    if (Tcw.matrix().hasNaN())
    {
        RCLCPP_WARN(this->get_logger(), "Invalid pose");
        return;
    }

    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f pos = Twc.translation();

    RCLCPP_INFO(
        this->get_logger(),
        "STEREO POS x=%.3f y=%.3f z=%.3f",
        pos.x(),
        pos.y(),
        pos.z()
    );
}
