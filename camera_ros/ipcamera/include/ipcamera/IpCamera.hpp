#ifndef _IPCAMERA_H_
#define _IPCAMERA_H_

#include <thread>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace camera {

class IpCamera : public rclcpp::Node {
public:
    explicit IpCamera(const std::string& node_name, const rclcpp::NodeOptions& options);
    explicit IpCamera(const rclcpp::NodeOptions& options);

    void configure();
    void initialize_parameters();
    void execute();

private:
    std::thread worker_thread;
    std::shared_mutex worker_thread_mutex;
    
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    std::string camera_calibration_file_param_;

    image_transport::CameraPublisher pub_;
    rclcpp::QoS qos_;
    std::chrono::milliseconds freq_ = 30ms;

    cv::VideoCapture cap_;
    std::string source_;
    int width_;
    int height_;

    std::string mat_type2encoding(int mat_type);

    void convert_frame_to_message(const cv::Mat& frame, size_t frame_id, 
            sensor_msgs::msg::Image& msg, sensor_msgs::msg::CameraInfo& camera_info_msg);

};

} // namespace camera

#endif // _IPCAMERA_H_
