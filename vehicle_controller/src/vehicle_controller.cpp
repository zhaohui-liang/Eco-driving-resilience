// vehicle_controller.cpp
#include "vehicle_controller/vehicle_controller.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <chrono>

VehicleController::VehicleController(rclcpp::Node* parent_node)
: logger_(parent_node->get_logger()),
  last_position_(0.0),
  last_speed_(0.0),
  last_yaw_rate_(0.0),
  traffic_light_position_(160.0),
  traffic_light_state_(0),
  time_to_next_phase_(0.0),
  trajectory_count_(0)
{
    trajectory_.clear();
    actual_trajectory_.clear();
}

void VehicleController::updatePosition(double position) {
    last_position_ = position;
    actual_trajectory_.push_back({last_position_, last_speed_, last_yaw_rate_});
    saveActualTrajectoryToFile("actual_trajectory.csv");
}

void VehicleController::updateSpeed(double speed) {
    last_speed_ = speed;
}

void VehicleController::updateYawRate(double yaw_rate) {
    last_yaw_rate_ = yaw_rate;
}

void VehicleController::setTrafficLightCondition(int state, double time_to_next) {
    traffic_light_state_ = state;
    time_to_next_phase_ = time_to_next;
}

void VehicleController::generateTrajectory() {
    trajectory_.clear();

    double expected_speed = 10.0;
    double distance = traffic_light_position_ - last_position_;
    double t_estimate = distance / std::max(1.0, last_speed_);
    double t_target = (t_estimate <= time_to_next_phase_) ? t_estimate : time_to_next_phase_ + 10.0;

    double a = (expected_speed + last_speed_) / pow(t_target, 2) - 2 * (distance) / pow(t_target, 3);
    double b = 3 * (distance) / pow(t_target, 2) - (2 * last_speed_ + expected_speed) / t_target;
    double c = last_speed_;
    double d = last_position_;

    for (double t = 0.0; t <= t_target; t += 0.1) {
        double pos = a * pow(t, 3) + b * pow(t, 2) + c * t + d;
        double spd = 3 * a * pow(t, 2) + 2 * b * t + c;
        trajectory_.push_back({pos, std::max(spd, 0.0), last_yaw_rate_});
    }

    savePredictedTrajectoryToFile("predicted_trajectory.csv");
    RCLCPP_INFO(logger_, "Trajectory generated with %zu points.", trajectory_.size());
    trajectory_count_++;
}

std::vector<TrajectoryPoint> VehicleController::getTrajectory() const {
    return trajectory_;
}

void VehicleController::savePredictedTrajectoryToFile(const std::string& filename) const {
    std::ofstream file;
    bool file_exists = std::filesystem::exists(filename);
    file.open(filename, std::ios::app);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger_, "Failed to open predicted trajectory file: %s", filename.c_str());
        return;
    }
    if (!file_exists) {
        file << "trajectory_id,timestamp,position,speed,angular_velocity\n";
    }
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    for (const auto& pt : trajectory_) {
        file << trajectory_count_ << ","
             << now_time << ","
             << std::fixed << std::setprecision(6)
             << pt.position << "," << pt.speed << "," << pt.angular_velocity << "\n";
    }
    file.close();
}

void VehicleController::saveActualTrajectoryToFile(const std::string& filename) const {
    std::ofstream file;
    bool file_exists = std::filesystem::exists(filename);
    file.open(filename, std::ios::app);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger_, "Failed to open actual trajectory file: %s", filename.c_str());
        return;
    }
    if (!file_exists) {
        file << "timestamp,position,speed,angular_velocity\n";
    }
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    const auto& pt = actual_trajectory_.back();
    file << now_time << ","
         << std::fixed << std::setprecision(6)
         << pt.position << "," << pt.speed << "," << pt.angular_velocity << "\n";
    file.close();
}
