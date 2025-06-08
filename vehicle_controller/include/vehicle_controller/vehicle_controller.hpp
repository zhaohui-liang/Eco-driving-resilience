// vehicle_controller.hpp
#pragma once

#include <vector>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

struct TrajectoryPoint {
    double position;
    double speed;
    double angular_velocity;
};

class VehicleController {
public:
    VehicleController(rclcpp::Node* parent_node);

    void updatePosition(double position);
    void updateSpeed(double speed);
    void updateYawRate(double yaw_rate);
    void setTrafficLightCondition(int state, int time_to_next);
    void generateTrajectory();
    const std::vector<TrajectoryPoint>& getTrajectory() const;
    double getLastSpeed() const;

private:
    void savePredictedTrajectoryToFile(const std::string& filename) const;
    void saveActualTrajectoryToFile(const std::string& filename) const;

    rclcpp::Logger logger_;
    double last_position_;
    double last_speed_;
    double last_yaw_rate_;
    double traffic_light_position_;
    double expected_speed_;
    int traffic_light_state_;
    double time_to_next_phase_;
    int trajectory_count_;
    double red_duration_;
    double green_duration_;
    double yellow_duration_;
    std::vector<TrajectoryPoint> trajectory_;
    std::vector<TrajectoryPoint> actual_trajectory_;
    double t_e_;
    double t_c_;
};
