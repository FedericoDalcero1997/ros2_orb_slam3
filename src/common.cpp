/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

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
    pubconfigackName = "/mono_py_driver/exp_settings_ack";
    subImgMsgName = "/mono_py_driver/img_msg";
    subTimestepMsgName = "/mono_py_driver/timestep_msg";
    subImuMsgName = "/zed/zed_node/imu/data_raw";

    RCLCPP_INFO(this->get_logger(), "subexperimentconfigName %s", subexperimentconfigName.c_str());
    RCLCPP_INFO(this->get_logger(), "pubconfigackName %s", pubconfigackName.c_str());
    RCLCPP_INFO(this->get_logger(), "subImgMsgName %s", subImgMsgName.c_str());
    RCLCPP_INFO(this->get_logger(), "subTimestepMsgName %s", subTimestepMsgName.c_str());

    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
        subexperimentconfigName,
        10,
        std::bind(&MonocularMode::experimentSetting_callback, this, _1)
    );

    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(
        pubconfigackName,
        10
    );

    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName,
        10,
        std::bind(&MonocularMode::Img_callback, this, _1)
    );

    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        subTimestepMsgName,
        10,
        std::bind(&MonocularMode::Timestep_callback, this, _1)
    );

    subImuMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        subImuMsgName,
        200,
        std::bind(&MonocularMode::Imu_callback, this, _1)
    );

    trackingThread_ = std::thread(&MonocularMode::trackingLoop, this);

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

MonocularMode::~MonocularMode()
{
    RCLCPP_INFO(this->get_logger(), "Destructor called");

    if (pAgent)
    {
        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            stopTracking_ = true;
        }
        frameCV_.notify_one();
        if (trackingThread_.joinable()) trackingThread_.join();
        RCLCPP_INFO(this->get_logger(), "Shutting down pAgent");
        pAgent->Shutdown();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "pAgent is null in destructor");
    }

    pass;
}

void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg)
{
    static bool already_initialized = false;

    RCLCPP_INFO(this->get_logger(), "experimentSetting_callback ENTERED");
    RCLCPP_INFO(this->get_logger(), "Received config string: %s", msg.data.c_str());

    if (already_initialized)
    {
        RCLCPP_WARN(this->get_logger(), "Ignoring duplicate experiment config");
        return;
    }

    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    receivedConfig = experimentConfig;

    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    RCLCPP_INFO(this->get_logger(), "Calling initializeVSLAM...");
    initializeVSLAM(experimentConfig);
    RCLCPP_INFO(this->get_logger(), "initializeVSLAM DONE, pAgent=%p", (void*)pAgent);

    auto message = std_msgs::msg::String();
    message.data = "ACK";

    std::cout << "Sent response: " << message.data.c_str() << std::endl;
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

    // IMPORTANT: do not append to settingsFilePath directly
    std::string fullSettingsFilePath = settingsFilePath + configString + ".yaml";

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", fullSettingsFilePath.c_str());

    sensorType = ORB_SLAM3::System::IMU_MONOCULAR;
    enablePangolinWindow = true;
    enableOpenCVWindow = true;

    RCLCPP_INFO(this->get_logger(), "Creating ORB_SLAM3::System...");
    pAgent = new ORB_SLAM3::System(vocFilePath, fullSettingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "ORB_SLAM3::System created, pAgent=%p", (void*)pAgent);

    std::cout << "MonocularMode node initialized" << std::endl;
}

void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg)
{
    timeStep = time_msg.data;
}

void MonocularMode::trackingLoop()
{
    while (true)
    {
        PendingFrame frame;
        {
            std::unique_lock<std::mutex> lock(frameMutex_);
            frameCV_.wait(lock, [this]{ return pendingFrame_.valid || stopTracking_; });
            if (stopTracking_) break;
            frame = std::move(pendingFrame_);
            pendingFrame_.valid = false;
        }

        Sophus::SE3f Tcw = pAgent->TrackMonocular(frame.image, frame.timestamp, frame.imuMeas);

        if (Tcw.matrix().hasNaN()) {
            RCLCPP_WARN(this->get_logger(), "Invalid pose, skipping");
            continue;
        }

        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f pos = Twc.translation();
        RCLCPP_INFO(this->get_logger(),
            "Camera position: x=%.3f y=%.3f z=%.3f | dist=%.3f | imu=%zu",
            pos.x(), pos.y(), pos.z(), pos.norm(), frame.imuMeas.size());
    }
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

        if (!cv_ptr)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_ptr is null after toCvCopy");
            return;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading image: %s", e.what());
        return;
    }

    double imageTime =
        static_cast<double>(msg.header.stamp.sec) +
        static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

    {
        std::lock_guard<std::mutex> lock(imuMutex);

        while (!imuBuf.empty())
        {
            const auto& imu = imuBuf.front();

            double imuTime =
                static_cast<double>(imu.header.stamp.sec) +
                static_cast<double>(imu.header.stamp.nanosec) * 1e-9;

            if (lastImageTime > 0.0 && imuTime <= lastImageTime)
            {
                imuBuf.pop_front();
                continue;
            }

            if (imuTime > imageTime)
                break;

            vImuMeas.emplace_back(
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
                imuTime
            );

            imuBuf.pop_front();
        }
    }

    const size_t MIN_IMU_MEAS = 3;
    if (lastImageTime > 0.0 && vImuMeas.size() < MIN_IMU_MEAS)
    {
        RCLCPP_WARN(this->get_logger(),
            "Too few IMU measurements (%zu < %zu), skipping frame",
            vImuMeas.size(), MIN_IMU_MEAS);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(frameMutex_);

        if (pendingFrame_.valid)
        {
            pendingFrame_.imuMeas.insert(
                pendingFrame_.imuMeas.end(),
                vImuMeas.begin(),
                vImuMeas.end()
            );
            pendingFrame_.image     = cv_ptr->image.clone();
            pendingFrame_.timestamp = imageTime;
        }
        else
        {
            pendingFrame_.image     = cv_ptr->image.clone();
            pendingFrame_.timestamp = imageTime;
            pendingFrame_.imuMeas   = std::move(vImuMeas);
            pendingFrame_.valid     = true;
            frameCV_.notify_one();
        }

        lastImageTime = imageTime;
    }
}

void MonocularMode::Imu_callback(const sensor_msgs::msg::Imu& msg)
{
    std::lock_guard<std::mutex> lock(imuMutex);
    imuBuf.push_back(msg);

    while (imuBuf.size() > 2000)
        imuBuf.pop_front();
}
