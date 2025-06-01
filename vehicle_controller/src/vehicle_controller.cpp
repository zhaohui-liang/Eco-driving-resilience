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
    parent_node->declare_parameter("expected_speed", 10.0);
    parent_node->declare_parameter("red_duration", 35.0);
    parent_node->declare_parameter("yellow_duration", 3.0);
    parent_node->declare_parameter("green_duration", 25.0);

    expected_speed_ = parent_node->get_parameter("expected_speed").as_double();
    red_duration_ = parent_node->get_parameter("red_duration").as_double();
    yellow_duration_ = parent_node->get_parameter("yellow_duration").as_double();
    green_duration_ = parent_node->get_parameter("green_duration").as_double();

    trajectory_.clear();
    actual_trajectory_.clear();
    // Initialize traffic light log
    std::ofstream file("traffic_light_log.csv");
    if (file.is_open()) {
        file << "Timestamp,State,TimeToNextPhase\n";
        file.close();
    }
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

double VehicleController::getLastSpeed() const {
    return last_speed_;
}

void VehicleController::setTrafficLightCondition(int state, int time_to_next) {
    traffic_light_state_ = state;

    double t  = time_to_next / 10.0; // Convert from tenths of seconds to seconds
    double cycle = red_duration_ + yellow_duration_ + green_duration_;
    
    double d = traffic_light_position_ - last_position_;
    // Calculate earliest arrival time t_e
    double t_e = 0.0;
    if (d > (pow(expected_speed_, 2) - pow(last_speed_, 2)) / 1.0) {
        t_e = (expected_speed_ - last_speed_) / 2.0 + (d - (pow(expected_speed_, 2) - pow(last_speed_, 2)) / 1.0) / expected_speed_;
    } else {
        t_e = sqrt(d + pow(last_speed_ / 2.0, 2)) - last_speed_ / 2.0;
    }

    switch (state) {
        case 3: // red
            time_to_next_phase_ = t;
            break;

        case 6: // green
            if (t > t_e) {
                time_to_next_phase_ = t_e-0.1;  // vehicle can make it through, offset 0.1 for GPS drift
            } else {
                time_to_next_phase_ = t + red_duration_ + yellow_duration_;  // wait for next green
            }
            break;

        case 8: // yellow
            time_to_next_phase_ = t + red_duration_;  // yellow then red then green
            break;

        default:
            RCLCPP_WARN(logger_, "Unknown traffic light state received: %d", state);
            time_to_next_phase_ = t;
            break;
    }


    // Clamp to within cycle
    if (time_to_next_phase_ > cycle) {
        time_to_next_phase_ = fmod(time_to_next_phase_, cycle);
    }
    // Log traffic light state
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    std::ofstream file("traffic_light_log.csv", std::ios::app);
    if (file.is_open()) {
        file << now_time << "," << state << "," << time_to_next_phase_ << "\n";
        file.close();
    }
    RCLCPP_INFO(logger_, "State: %d | time_to_next_phase: %.2f s", state, time_to_next_phase_);
}


void VehicleController::generateTrajectory() {
    trajectory_.clear();
    double d = traffic_light_position_ - last_position_;
    if (d <= 0.0) {
        if (last_speed_ <= 0.01) {
            RCLCPP_WARN(logger_, "Vehicle already stopped. No deceleration needed.");
            return;
        }
        RCLCPP_WARN(logger_, "Vehicle has reached or passed the traffic light. Decelerating to stop.");
        double decel_duration = std::max(last_speed_ / 1.0, 1.0);  // at least 1 second, assume 1 m/s² decel
        for (double t = 0.0; t <= decel_duration; t += 0.01) {
            double spd = std::max(last_speed_ - t * 1.0, 0.0);  // linear deceleration
            double pos = last_position_ + last_speed_ * t - 0.5 * 1.0 * t * t;
            trajectory_.push_back({pos, spd, last_yaw_rate_});
        }
        // Append extra points to ensure vehicle is stationary
        constexpr double stopped_extension_time = 2.0;  // 2 seconds of "stopped" time
        for (double t = 0.0; t <= stopped_extension_time; t += 0.01) {
            trajectory_.push_back({last_position_, 0.0, last_yaw_rate_});
        }
        savePredictedTrajectoryToFile("predicted_trajectory.csv");
        RCLCPP_INFO(logger_, "Deceleration trajectory generated (d <= 0). Duration: %.2f s", decel_duration);
        trajectory_count_++;
        return;
    }
    double t_e = 0.0;
    if (d > (pow(expected_speed_, 2) - pow(last_speed_, 2)) / 1.0) {
        t_e = (expected_speed_ - last_speed_) / 2.0 + (d - (pow(expected_speed_, 2) - pow(last_speed_, 2)) / 1.0) / expected_speed_;
    } else {
        t_e = sqrt(d + pow(last_speed_ / 2.0, 2)) - last_speed_ / 2.0;
    }
    double t_c = 3.0 * d / (last_speed_ + expected_speed_ - sqrt(last_speed_ * expected_speed_));

    std::vector<TrajectoryPoint> temp_trajectory;
    std::vector<double> t_values;
    for (double t = 0.0; t <= time_to_next_phase_; t += 0.01) {
        t_values.push_back(t);
    }

    if (time_to_next_phase_ <= t_e) {
        for (double t : t_values) {
            double pos = expected_speed_ * t;
            double spd = expected_speed_;
            temp_trajectory.push_back({pos, spd, last_yaw_rate_});
        }
    } else if (time_to_next_phase_ < t_c) {
        double a = 2 * d / pow(time_to_next_phase_, 3) + (expected_speed_ + last_speed_) / pow(time_to_next_phase_, 2);
        double b = 3 * d / pow(time_to_next_phase_, 2) - (2 * last_speed_ + expected_speed_) / time_to_next_phase_;
        double c = last_speed_;
        double x0 = last_position_;

        for (double t : t_values) {
            double pos = a * pow(t, 3) + b * pow(t, 2) + c * t + x0;
            double spd = 3 * a * pow(t, 2) + 2 * b * t + c;
            spd = std::min(spd, expected_speed_);
            temp_trajectory.push_back({pos, spd, last_yaw_rate_});
        }
    } else {
        double t_w = time_to_next_phase_ - t_c;
        double a = 2 * d / pow(t_c, 3) + (expected_speed_ + last_speed_) / pow(t_c, 2);
        double b = 3 * d / pow(t_c, 2) - (2 * last_speed_ + expected_speed_) / t_c;
        double c = last_speed_;
        double x0 = last_position_;

        double t_s = -b / (3 * a);
        double x_s = a * pow(t_s, 3) + b * pow(t_s, 2) + c * t_s + x0;

        size_t idx_t_s = 0;
        size_t idx_t_sw = 0;
        for (size_t i = 0; i < t_values.size(); ++i) {
            if (t_values[i] > t_s && idx_t_s == 0) idx_t_s = i;
            if (t_values[i] > t_s + t_w && idx_t_sw == 0) idx_t_sw = i;
        }

        for (size_t i = 0; i < idx_t_s; ++i) {
            double t = t_values[i];
            double pos = a * pow(t, 3) + b * pow(t, 2) + c * t + x0;
            double spd = 3 * a * pow(t, 2) + 2 * b * t + c;
            spd = std::min(spd, expected_speed_);
            temp_trajectory.push_back({pos, spd, last_yaw_rate_});
        }

        for (size_t i = idx_t_s; i < idx_t_sw; ++i) {
            temp_trajectory.push_back({x_s, 0.0, last_yaw_rate_});
        }

        for (size_t i = idx_t_sw; i < t_values.size(); ++i) {
            double t = t_values[i] - t_w;
            double pos = a * pow(t, 3) + b * pow(t, 2) + c * t + x0;
            double spd = 3 * a * pow(t, 2) + 2 * b * t + c;
            spd = std::min(spd, expected_speed_);
            temp_trajectory.push_back({pos, spd, last_yaw_rate_});
        }
    }

    trajectory_ = temp_trajectory;
    savePredictedTrajectoryToFile("predicted_trajectory.csv");
    RCLCPP_INFO(logger_, "Trajectory generated with %zu points.", trajectory_.size());
    trajectory_count_++;
}

const std::vector<TrajectoryPoint>& VehicleController::getTrajectory() const {
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
