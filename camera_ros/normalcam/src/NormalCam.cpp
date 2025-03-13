#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <filesystem>

#include <rclcpp/parameter.hpp>
#include "normalcam/NormalCam.hpp"

namespace camera {

NormalCam::NormalCam(const std::string& node_name, const rclcpp::NodeOptions& options, 
        const std::string &camera_source)
    : Node(node_name, options) {

    frame_id_ = this->declare_parameter("frame_id", "normalcam");
    camera_source_ = this->declare_parameter("camera_source_", camera_source);

    width_ = this->declare_parameter("image_width", 640);
    height_ = this->declare_parameter("image_height", 480);   
    RCLCPP_INFO(this->get_logger(), "camera %s.", camera_source_);
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    this -> cam_info_pub_ = image_transport::create_camera_publisher(
            this, "~/image_raw", custom_qos_profile);
    
    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

    // auto camera_calibration_file_param_ = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
    // cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);
}

NormalCam::NormalCam(const rclcpp::NodeOptions& options)
    : NormalCam::NormalCam("normalCam", options, "0") {

}

bool NormalCam::camera_init(const std::string &source) {
    cap_.open(std::stoi(source));
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "camera opened successfully.");
    return true;
}

 // 启动摄像头
 void NormalCam::start() {
    std::string source = this->get_parameter("camera_source_").as_string();
    if (!camera_init(source)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera!");
        return;
    }
    start_callback(); // 启动回调
}

void NormalCam::start_callback() {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33), // 30 FPS
        std::bind(&NormalCam::callback, this)
    );
}


void NormalCam::callback() {
    
    cap_ >> frame;
    
    if(!frame.empty()) {
        auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(
            cinfo_manager_ -> getCameraInfo());
        auto msg_ = std::make_unique<sensor_msgs::msg::Image>();
        msg_ -> is_bigendian = false;

        convert_frame_to_message(frame, frame_id_, *msg_, *camera_info_msg);
        this -> cam_info_pub_.publish(std::move(msg_), camera_info_msg);
    }
}


void NormalCam::convert_frame_to_message(
    const cv::Mat& frame, std::string frame_id,
    sensor_msgs::msg::Image& msg, 
    sensor_msgs::msg::CameraInfo& camera_info_msg) {
    
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = "bgr8";
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);

    rclcpp::Time timestamp = this -> get_clock() -> now();

    msg.header.frame_id = frame_id;
    msg.header.stamp = timestamp;
    camera_info_msg.header.frame_id = frame_id;
    camera_info_msg.header.stamp = timestamp;
}



} // namespace camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera::NormalCam)
