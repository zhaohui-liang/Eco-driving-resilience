// vehicle_node.cpp
// Entry point for the ROS2 vehicle controller node

#include <rclcpp/rclcpp.hpp>
#include "vehicle_optimal/vehicle_optimal.hpp"
#include "vehicle_optimal/signal_io.hpp"

class VehicleNode : public rclcpp::Node {
public:
    VehicleNode() : Node("vehicle_controller_node") {
        RCLCPP_INFO(this->get_logger(), "Vehicle controller node started.");

        controller_ = std::make_shared<VehicleController>(this);
        signal_io_ = std::make_shared<SignalIO>(this, controller_);
    }

private:
    std::shared_ptr<VehicleController> controller_;
    std::shared_ptr<SignalIO> signal_io_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleNode>());
    rclcpp::shutdown();
    return 0;
}
