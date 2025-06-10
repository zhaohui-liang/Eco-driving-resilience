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
    using namespace std::chrono;
    auto start_time = high_resolution_clock::now();

    const double x0 = last_position_;
    const double v0 = last_speed_;
    const double xf = traffic_light_position_;
    const double vf = expected_speed_;
    const double tf = time_to_next_phase_;
    const double dt = 0.1;

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

bool VehicleController::solveEcoDrivingOptimization(double x0, double v0,
                                                    double xf, double vf,
                                                    double tf, double dt,
                                                    std::vector<double>& x_out,
                                                    std::vector<double>& v_out) {
    const int N = static_cast<int>(tf / dt);
    const int n_vars = 3 * N + 2;  // x[0..N], v[0..N], a[0..N-1]
    const int n_constraints = 2 * N + 4;

    const double alpha = 0.15, beta = 0.0025, gamma = 0.00006;
    const double delta = 0.00035, epsilon = 0.0004;
    const double v_max = 10.0;
    const double a_min = -2.0, a_max = 2.0;

    // Decision variables: [x_0..x_N, v_0..v_N, a_0..a_N-1]
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

    std::vector<Eigen::Triplet<double>> tripletList;

    // Fill cost function
    for (int i = 0; i < N; ++i) {
        int v_idx = N + 1 + i;
        int a_idx = 2 * (N + 1) + i;

        // Quadratic terms
        tripletList.emplace_back(v_idx, v_idx, gamma);
        tripletList.emplace_back(a_idx, a_idx, epsilon);
        tripletList.emplace_back(v_idx, a_idx, delta / 2.0);
        tripletList.emplace_back(a_idx, v_idx, delta / 2.0);

        // Linear term
        q(v_idx) += beta;
    }

    P.setFromTriplets(tripletList.begin(), tripletList.end());

    // Constraints: A*x = b
    Eigen::SparseMatrix<double> A(n_constraints, n_vars);
    Eigen::VectorXd l = Eigen::VectorXd::Zero(n_constraints);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_constraints);
    std::vector<Eigen::Triplet<double>> A_triplets;

    int row = 0;

    // Initial conditions
    A_triplets.emplace_back(row, 0, 1.0); l(row) = x0; u(row) = x0; row++; // x0
    A_triplets.emplace_back(row, N + 1, 1.0); l(row) = v0; u(row) = v0; row++; // v0

    // Final conditions
    A_triplets.emplace_back(row, N, 1.0); l(row) = xf; u(row) = xf; row++; // x_N
    A_triplets.emplace_back(row, 2 * (N + 1) - 1, 1.0); l(row) = vf; u(row) = vf; row++; // v_N

    // Dynamics constraints
    for (int i = 0; i < N; ++i) {
        int xi = i, xi1 = i + 1;
        int vi = N + 1 + i;
        int ai = 2 * (N + 1) + i;

        // x_{i+1} = x_i + v_i*dt + 0.5*a_i*dt^2
        A_triplets.emplace_back(row, xi1, 1.0);
        A_triplets.emplace_back(row, xi, -1.0);
        A_triplets.emplace_back(row, vi, -dt);
        A_triplets.emplace_back(row, ai, -0.5 * dt * dt);
        l(row) = 0; u(row) = 0; row++;

        // v_{i+1} = v_i + a_i*dt
        int vi1 = vi + 1;
        A_triplets.emplace_back(row, vi1, 1.0);
        A_triplets.emplace_back(row, vi, -1.0);
        A_triplets.emplace_back(row, ai, -dt);
        l(row) = 0; u(row) = 0; row++;
    }

    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    // Setup OSQP problem
    csc* P_csc = osqp_sparse(P.rows(), P.cols(), P.nonZeros(), P.valuePtr(), P.innerIndexPtr(), P.outerIndexPtr());
    csc* A_csc = osqp_sparse(A.rows(), A.cols(), A.nonZeros(), A.valuePtr(), A.innerIndexPtr(), A.outerIndexPtr());

    OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));

    osqp_set_default_settings(settings);
    settings->verbose = false;

    data->n = n_vars;
    data->m = n_constraints;
    data->P = P_csc;
    data->q = q.data();
    data->A = A_csc;
    data->l = l.data();
    data->u = u.data();

    OSQPWorkspace* work = osqp_setup(data, settings);
    osqp_solve(work);

    bool success = work->info->status_val == OSQP_SOLVED;

    if (success) {
        x_out.resize(N + 1);
        v_out.resize(N + 1);
        for (int i = 0; i <= N; ++i) {
            x_out[i] = work->solution->x[i];
            v_out[i] = work->solution->x[N + 1 + i];
        }
    }

    osqp_cleanup(work);
    c_free(settings);
    c_free(data);
    c_free(P_csc);
    c_free(A_csc);

    return success;
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
