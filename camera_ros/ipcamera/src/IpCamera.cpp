#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/parameter.hpp>
#include "ipcamera/IpCamera.hpp"

namespace camera {

IpCamera::IpCamera(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options), 
      qos_(rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()) {
    
    RCLCPP_INFO(this -> get_logger(), "namespace: %s", this -> get_namespace());
    RCLCPP_INFO(this -> get_logger(), "node_name: %s", this -> get_name());

    this -> initialize_parameters();
    this -> configure();

    this -> pub_ = image_transport::create_camera_publisher(
            this, "~/image_raw", qos_.get_rmw_qos_profile());
    
    this -> execute();
}

IpCamera::IpCamera(const rclcpp::NodeOptions& options)
    : IpCamera::IpCamera("ipcamera", options) {}

void IpCamera::configure() {
    rclcpp::Logger node_logger = this -> get_logger();

    this -> get_parameter<std::string>("rtsp_uri", source_);
    this -> get_parameter<std::string>("camera_calibration_file", 
            camera_calibration_file_param_);
    this -> get_parameter<int>("image_width", width_);
    this -> get_parameter<int>("image_height", height_);

    RCLCPP_INFO(node_logger, "rtsp_uri: %s", source_.c_str());
    RCLCPP_INFO(node_logger, "camera_calibration_file_param: %s", 
            camera_calibration_file_param_.c_str());
    RCLCPP_INFO(node_logger, "image_width: %d", width_);
    RCLCPP_INFO(node_logger, "image_height: %d", height_);

    this -> cap_.open(source_);
    if(!this -> cap_.isOpened()) {
        RCLCPP_ERROR(node_logger, "Could not open video stream");
        throw std::runtime_error("Could not open video stream");
    }

    this -> cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double> (width_));
    this -> cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double> (height_));
    
    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    if(cinfo_manager_ -> validateURL(camera_calibration_file_param_)) {
        cinfo_manager_ -> loadCameraInfo(camera_calibration_file_param_);
    }
    else {
        RCLCPP_WARN(node_logger, "CameraInfo URL not valid.");
        RCLCPP_WARN(node_logger, "URL is %s", camera_calibration_file_param_.c_str());
    }
}


void IpCamera::initialize_parameters() {
    rcl_interfaces::msg::ParameterDescriptor rtsp_uri_descriptor;
    rtsp_uri_descriptor.name = "rtsp_uri";
    rtsp_uri_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    rtsp_uri_descriptor.description = "RTSP URI of the IP camera.";
    rtsp_uri_descriptor.additional_constraints = "SHoule be of the form 'rtsp://";
    this -> declare_parameter("rtsp_uri", "", rtsp_uri_descriptor);

    rcl_interfaces::msg::ParameterDescriptor camera_calibration_file_descriptor;
    camera_calibration_file_descriptor.name = "camera_calibration_file";
    camera_calibration_file_descriptor.type = 
        rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this -> declare_parameter(
        "camera_calibration_file", "", camera_calibration_file_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_width_descriptor;
    image_width_descriptor.name = "image_width";
    image_width_descriptor.type = 
        rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this -> declare_parameter("image_width", 640, image_width_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_heigth_descriptor;
    image_width_descriptor.name = "image_heigth";
    image_width_descriptor.type = 
        rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this -> declare_parameter("image_width", 480, image_heigth_descriptor);
}

void IpCamera::execute() {
    rclcpp::Rate loop_rate(freq_);
    
    auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(
            cinfo_manager_ -> getCameraInfo());
            
    cv::Mat frame;
    size_t frame_id = 0;

    while(rclcpp::ok()) {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg -> is_bigendian = false;

        this -> cap_ >> frame;
        if(!frame.empty()) {
            convert_frame_to_message(frame, frame_id, *msg, *camera_info_msg);
            this -> pub_.publish(std::move(msg), camera_info_msg);
            ++frame_id;
        }
        loop_rate.sleep();
    }
}

std::string IpCamera::mat_type2encoding(int mat_type) {
    switch (mat_type) {
        case CV_8UC1 :
            return "mono8";
        case CV_8UC3 :
            return "bgr8";
        case CV_16SC1 :
            return "mono16";
        case CV_8UC4 :
            return "rgbd8";
        default :
            throw std::runtime_error("Unsupported encoding type");
    }
}


void IpCamera::convert_frame_to_message(
    const cv::Mat& frame, size_t frame_id,
    sensor_msgs::msg::Image& msg, 
    sensor_msgs::msg::CameraInfo& camera_info_msg) {
    
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);

    rclcpp::Time timestamp = this -> get_clock() -> now();

    msg.header.frame_id = std::to_string(frame_id);
    msg.header.stamp = timestamp;
    camera_info_msg.header.frame_id = std::to_string(frame_id);
    camera_info_msg.header.stamp = timestamp;
}

} // namespace camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera::IpCamera)
