#include <memory>

#include "normalcam/NormalCam.hpp"

int main(int argc, char* argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto camera_node = std::make_shared<camera::NormalCam>("mycamera", options);
    exec.add_node(camera_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

