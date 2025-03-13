#ifndef _NORMALCAM_H_
#define _NORMALCAM_H_

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"


using namespace std::chrono_literals;
namespace camera {

class NormalCam : public rclcpp::Node {

public:
    explicit NormalCam(const std::string& node_name, const rclcpp::NodeOptions& options, 
        const std::string &camera_source);
    explicit NormalCam(const rclcpp::NodeOptions& options);

    bool camera_init(const std::string &source);

    // 启动相机
    void start();

protected:
    // 启动回调函数
    void start_callback();

    // 回调函数
    void callback();

private:
    
    cv::Mat frame;
    cv::VideoCapture cap_;
    int width_;
    int height_;
    std::string frame_id_;
    std::string camera_source_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    image_transport::CameraPublisher cam_info_pub_;

    void convert_frame_to_message(const cv::Mat& frame,std::string frame_id, 
            sensor_msgs::msg::Image& msg, sensor_msgs::msg::CameraInfo& camera_info_msg);
};

} // namespace camera


#endif  // _NORMALCAM_H_