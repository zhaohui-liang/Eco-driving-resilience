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
    if (d > (pow(expected_speed_, 2) - pow(last_speed_, 2)) / 1.0) {
        t_e_ = (expected_speed_ - last_speed_) / 2.0 + 
              (d - (pow(expected_speed_, 2) - pow(last_speed_, 2)) / 1.0) / expected_speed_;
    } else {
        t_e_ = sqrt(d + pow(last_speed_ / 2.0, 2)) - last_speed_ / 2.0;
    }

    // Calculate critical time t_c
    t_c_ = 3.0 * d / (last_speed_ + expected_speed_ - sqrt(last_speed_ * expected_speed_));

    switch (state) {
        case 3: // red
            time_to_next_phase_ = t;
            break;

        case 6: // green
            if (t > t_e_) {
                time_to_next_phase_ = t_e_;
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

    // Clamp within cycle time
    if (time_to_next_phase_ > cycle) {
        time_to_next_phase_ = fmod(time_to_next_phase_, cycle);
    }

    // Log to CSV (misc_log.csv)
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    // Check if file exists and is empty
    bool write_header = false;
    if (!std::filesystem::exists("misc_log.csv") || std::filesystem::file_size("misc_log.csv") == 0) {
        write_header = true;
    }
    std::ofstream file("misc_log.csv", std::ios::app);
    if (file.is_open()) {
        if (write_header) {
        file << "timestamp,state,distance,last_speed,t_e_,t_c_,time_to_next_phase\n";
        }
        file << now_time << "," << state << "," << d << "," << last_speed_ << ","
             << t_e_ << "," << t_c_ << "," << time_to_next_phase_ << "\n";
        file.close();
    } else {
        RCLCPP_WARN(logger_, "Failed to open misc_log.csv for writing.");
    }

    RCLCPP_INFO(logger_, "State: %d | d: %.2f | v: %.2f | t_e_: %.2f | t_c_: %.2f | t_phase: %.2f",
                state, d, last_speed_, t_e_, t_c_, time_to_next_phase_);
}



void VehicleController::generateTrajectory() {
    constexpr double EPSILON = 1e-2;
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
        constexpr double stopped_extension_time = 2.0;
        for (double t = 0.0; t <= stopped_extension_time; t += 0.01) {
            trajectory_.push_back({last_position_, 0.0, last_yaw_rate_});
        }
        savePredictedTrajectoryToFile("predicted_trajectory.csv");
        RCLCPP_INFO(logger_, "Deceleration trajectory generated (d <= 0). Duration: %.2f s", decel_duration);
        trajectory_count_++;
        return;
    }

    double t_e = t_e_;
    double t_c = t_c_;

    std::vector<TrajectoryPoint> temp_trajectory;
    std::vector<double> t_values;
    for (double t = 0.0; t <= time_to_next_phase_; t += 0.01) {
        t_values.push_back(t);
    }

    // Case 1: Constant speed
    if (time_to_next_phase_ <= t_e + EPSILON) {
        for (double t : t_values) {
            double pos = expected_speed_ * t;
            double spd = expected_speed_;
            temp_trajectory.push_back({pos, spd, last_yaw_rate_});
        }
    }
    // Case 2: Polynomial interpolation deceleration
    else if (time_to_next_phase_ < t_c - EPSILON) {
        double a = 2 * d / pow(time_to_next_phase_, 3) + 
                   (expected_speed_ + last_speed_) / pow(time_to_next_phase_, 2);
        double b = 3 * d / pow(time_to_next_phase_, 2) - 
                   (2 * last_speed_ + expected_speed_) / time_to_next_phase_;
        double c = last_speed_;
        double x0 = last_position_;

        for (double t : t_values) {
            double pos = a * pow(t, 3) + b * pow(t, 2) + c * t + x0;
            double spd = 3 * a * pow(t, 2) + 2 * b * t + c;
            spd = std::min(spd, expected_speed_);
            temp_trajectory.push_back({pos, spd, last_yaw_rate_});
        }
    }
    // Case 3: Wait + restart
    else {
        double t_w = time_to_next_phase_ - t_c;
        double a = 2 * d / pow(t_c, 3) + 
                   (expected_speed_ + last_speed_) / pow(t_c, 2);
        double b = 3 * d / pow(t_c, 2) - 
                   (2 * last_speed_ + expected_speed_) / t_c;
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
    RCLCPP_ERROR(logger_, "time_to_next_phase:%.2f,t_e:%.2f", time_to_next_phase_,t_e);
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
