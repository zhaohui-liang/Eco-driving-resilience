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
            time_to_next_phase_ = max(t,t_e_);
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
    auto start_time = high_resolution_clock::now();

    const double x0 = last_position_;
    const double v0 = last_speed_;
    const double xf = traffic_light_position_;
    const double vf = expected_speed_;
    const double tf = time_to_next_phase_;
    const double dt = 0.01;

    std::vector<double> x_opt, v_opt;

    bool success = solveEcoDrivingOptimization(x0, v0, xf, vf, tf, dt, x_opt, v_opt);

    if (!success) {
        RCLCPP_ERROR(logger_, "Eco-driving optimization failed.");
        return;
    }

    trajectory_.clear();
    for (size_t i = 0; i < x_opt.size(); ++i) {
        trajectory_.push_back({x_opt[i], v_opt[i], last_yaw_rate_});
    }

    savePredictedTrajectoryToFile("predicted_trajectory.csv");

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(logger_, "Eco-driving trajectory generated in %ld ms", duration);
    trajectory_count_++;
}

bool VehicleController::solveEcoDrivingOptimization(
    double x0, double v0,
    double xf, double vf,
    double tf, double dt,
    std::vector<double>& x_out,
    std::vector<double>& v_out)
{
    int N = static_cast<int>(tf / dt);
    const int n_var = 2 * N; // [x1...xN, v1...vN]
    if (N < 2) {
    RCLCPP_ERROR(logger_, "Insufficient horizon steps (N = %d). tf = %.2f, dt = %.2f", N, tf, dt);
    return false;
    }
    RCLCPP_INFO(logger_, "solveEcoDrivingOptimization: x0=%.2f, v0=%.2f, xf=%.2f, vf=%.2f, tf=%.2f, dt=%.2f, N=%d",
            x0, v0, xf, vf, tf, dt, N);
    // Hessian: penalize acceleration squared => differences in velocity
    Eigen::SparseMatrix<double> hessian(n_var, n_var);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n_var);

    std::vector<Eigen::Triplet<double>> hessian_triplets;
    double w_acc = 10.0;
    for (int i = 0; i < N - 1; ++i) {
        int idx_i = N + i;
        int idx_ip1 = N + i + 1;
        hessian_triplets.push_back({idx_i, idx_i, w_acc});
        hessian_triplets.push_back({idx_ip1, idx_ip1, w_acc});
        hessian_triplets.push_back({idx_i, idx_ip1, -w_acc});
        hessian_triplets.push_back({idx_ip1, idx_i, -w_acc});
    }
    hessian.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());

    // Constraints: x0, v0, dynamics: x_{k+1} = x_k + v_k*dt
    const int n_con = 2 * (N - 1) + 2; // dynamics + initial + final
    Eigen::SparseMatrix<double> A(n_con, n_var);
    Eigen::VectorXd lower_bound(n_con);
    Eigen::VectorXd upper_bound(n_con);

    std::vector<Eigen::Triplet<double>> A_triplets;

    int row = 0;
    // Dynamics constraints
    for (int i = 0; i < N - 1; ++i) {
        // x_{i+1} - x_i - dt * v_i = 0
        A_triplets.push_back({row, i, -1.0});
        A_triplets.push_back({row, i + 1, 1.0});
        A_triplets.push_back({row, N + i, -dt});
        lower_bound[row] = 0.0;
        upper_bound[row] = 0.0;
        ++row;
    }

    // Initial constraints
    A_triplets.push_back({row, 0, 1.0});        // x0
    lower_bound[row] = x0;
    upper_bound[row] = x0;
    ++row;

    A_triplets.push_back({row, N + 0, 1.0});    // v0
    lower_bound[row] = v0;
    upper_bound[row] = v0;
    ++row;

    // Final constraints
    A_triplets.push_back({row, N - 1, 1.0});    // xN
    lower_bound[row] = xf;
    upper_bound[row] = xf;
    ++row;

    A_triplets.push_back({row, 2 * N - 1, 1.0}); // vN
    lower_bound[row] = vf;
    upper_bound[row] = vf;
    ++row;

    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    // Setup solver
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(n_var);
    solver.data()->setNumberOfConstraints(n_con);

    if (!solver.data()->setHessianMatrix(hessian)) return false;
    if (!solver.data()->setGradient(gradient)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(A)) return false;
    if (!solver.data()->setLowerBound(lower_bound)) return false;
    if (!solver.data()->setUpperBound(upper_bound)) return false;

    if (!solver.initSolver()) return false;
    auto result = solver.solveProblem();
    if (result != OsqpEigen::ErrorExitFlag::NoError) {
    RCLCPP_ERROR(logger_, "OSQP solve failed with exit code: %d", static_cast<int>(result));
    return false;
    }

    Eigen::VectorXd sol = solver.getSolution();
    x_out.clear();
    v_out.clear();
    for (int i = 0; i < N; ++i) {
        x_out.push_back(sol[i]);
        v_out.push_back(sol[N + i]);
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
