#ifndef _NORMALCAM_H_
#define _NORMALCAM_H_

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"


using namespace std::chrono_literals;

// enum camera_type {
//     usb,
//     rtsp
// };

namespace camera {

class NormalCam : public rclcpp::Node {

public:
    explicit NormalCam(const std::string& node_name, const rclcpp::NodeOptions& options);
    explicit NormalCam(const rclcpp::NodeOptions& options);


private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds freq_ = 30ms;
    
    cv::Mat frame;
    cv::Mat flipped_frame;
    cv::VideoCapture cap_;
    bool is_flipped;
    bool use_usb_;
    int camera_id;
    int width_;
    int height_;
    std::string rtsp_;
    std::string frame_id_;
    
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    image_transport::CameraPublisher cam_info_pub_;

    void convert_frame_to_message(const cv::Mat& frame,std::string frame_id, 
            sensor_msgs::msg::Image& msg, sensor_msgs::msg::CameraInfo& camera_info_msg);

    void ImageCallback();

};

} // namespace camera


#endif  // _NORMALCAM_H_