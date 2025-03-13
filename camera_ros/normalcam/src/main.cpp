#include "normalcam/NormalCam.hpp"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto usbnode = std::make_shared<camera::NormalCam>("usbcam", options, "0");
    usbnode->start(); // 显式启动摄像头和回调
    rclcpp::spin(usbnode);
    rclcpp::shutdown();
    return 0;
}

