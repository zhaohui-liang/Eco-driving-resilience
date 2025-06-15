// vehicle_controller.cpp
#include "vehicle_optimal/vehicle_optimal.hpp"
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
            time_to_next_phase_ = std::max(t,t_e_);
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
    using namespace std::chrono;
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
    
    auto start_time = high_resolution_clock::now();

    const double x0 = last_position_;
    const double v0 = last_speed_;
    const double xf = traffic_light_position_;
    const double vf = expected_speed_;
    const double tf = time_to_next_phase_;
    const double dt = 0.1;

    std::vector<double> x_opt, v_opt;

    bool success = solveEcoDrivingOptimization(x0, v0, xf, vf, tf, vf+1.0, x_opt, v_opt);

    if (!success) {
        RCLCPP_ERROR(logger_, "Eco-driving optimization failed.");
        return;
    }

    trajectory_.clear();
    const double interp_dt = 0.01;  // Interpolated output frequency (100 Hz)
    for (size_t i = 0; i < x_opt.size() - 1; ++i) {
        double t0 = i * dt;
        double t1 = (i + 1) * dt;

        double x0 = x_opt[i];
        double x1 = x_opt[i + 1];

        double v0 = v_opt[i];
        double v1 = v_opt[i + 1];

        for (double t = t0; t < t1; t += interp_dt) {
            double alpha = (t - t0) / (t1 - t0);
            double x_interp = (1 - alpha) * x0 + alpha * x1;
            double v_interp = (1 - alpha) * v0 + alpha * v1;
            trajectory_.push_back({x_interp, v_interp, last_yaw_rate_});
        }
    }
    // Add the final point explicitly to avoid rounding issues
    trajectory_.push_back({x_opt.back(), v_opt.back(), last_yaw_rate_});
    savePredictedTrajectoryToFile("predicted_trajectory.csv");

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(logger_, "Eco-driving trajectory generated in %ld ms", duration);
    trajectory_count_++;
}

bool VehicleController::solveEcoDrivingOptimization(
    double x0, double v0, double xf, double vf, double tp, double vmax,
    std::vector<double>& x_out, std::vector<double>& v_out){
    using namespace casadi;
    double dt = 0.1;
    int N = static_cast<int>(tp / dt) + 1;

    SX X = SX::sym("X", 4, N);  // x, v, a, j

    SX x = X(0, Slice());
    SX v = X(1, Slice());
    SX a = X(2, Slice());
    SX j = X(3, Slice());

    // Objective: fuel-related cost
    SX J = 0;
    double alpha = 0.15;
    double beta = 0.0025;
    double gamma = 0.00006;
    double delta = 0.00035;
    double epsilon = 0.0004;
    for (int k = 0; k < N - 1; ++k) {
        J += alpha + beta * v(k) + gamma * pow(v(k), 2)
           + delta * v(k) * a(k) + epsilon * pow(a(k), 2);
    }

    // Constraints
    std::vector<SX> g;

    // Initial and final conditions
    g.push_back(x(0) - x0);
    g.push_back(v(0) - v0);
    g.push_back(x(N - 1) - xf);  // or xf
    g.push_back(v(N - 1) - vf);

    // Dynamics constraints
    for (int k = 0; k < N - 1; ++k) {
        g.push_back(x(k + 1) - (x(k) + v(k) * dt + 0.5 * a(k) * dt * dt));
        g.push_back(v(k + 1) - (v(k) + a(k) * dt));
        g.push_back(a(k + 1) - (a(k) + j(k) * dt));
    }

    // Flatten all decision variables into a vector
    SX Z = reshape(X, 4 * N, 1);

    // Create NLP solver
    Function solver = nlpsol("solver", "ipopt", {
        {"x", Z},
        {"f", J},
        {"g", vertcat(g)}
    }, {
        {"ipopt.print_level", 0},
        {"print_time", false}
    });

    // Set bounds
    DM lbz = DM::zeros(4 * N);
    DM ubz = DM::zeros(4 * N);
    for (int i = 0; i < N; ++i) {
        lbz(4 * i + 0) = -inf;         // x
        ubz(4 * i + 0) = inf;
        lbz(4 * i + 1) = 0.0;          // v
        ubz(4 * i + 1) = vmax;
        lbz(4 * i + 2) = -2.0;         // a
        ubz(4 * i + 2) = 2.0;
        lbz(4 * i + 3) = -0.5;         // j
        ubz(4 * i + 3) = 0.5;
    }

    // Constraint bounds (all equality)
    DM lbg = DM::zeros(g.size());
    DM ubg = DM::zeros(g.size());

    // Solve the NLP
    std::map<std::string, DM> arg = {
        {"x0", DM::zeros(4 * N)},
        {"lbx", lbz},
        {"ubx", ubz},
        {"lbg", lbg},
        {"ubg", ubg}
    };

    auto res = solver(arg);
    DM sol = res.at("x");

    // Extract x and v
    x_out.resize(N);
    v_out.resize(N);
    for (int k = 0; k < N; ++k) {
        x_out[k] = static_cast<double>(sol(4 * k + 0));
        v_out[k] = static_cast<double>(sol(4 * k + 1));
    }

    return true;
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
