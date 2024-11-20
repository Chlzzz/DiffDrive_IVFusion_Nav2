#include <memory>

#include "ipcamera/IpCamera.hpp"

int main(int argc, char* argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto ipcamera_node = std::make_shared<camera::IpCamera>("ipcamera", options);

    exec.add_node(ipcamera_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

