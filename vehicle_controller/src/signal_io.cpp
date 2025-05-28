// signal_io.cpp (updated with TwistStamped)
#include "vehicle_controller/signal_io.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <mutex>

SignalIO::SignalIO(rclcpp::Node* node, std::shared_ptr<VehicleController> controller)
: node_(node), controller_(controller)
{
    vel_sub_ = node_->create_subscription<novatel_oem7_msgs::msg::INSPVAX>(
        "/bynav/inspvax", 10,
        std::bind(&SignalIO::imuCallback, this, std::placeholders::_1));

    pos_sub_ = node_->create_subscription<novatel_oem7_msgs::msg::BESTGNSSPOS>(
        "/bynav/bestgnsspos", 10,
        std::bind(&SignalIO::gpsCallback, this, std::placeholders::_1));
    
    speed_sub_ = node_->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
    "/bynav/bestvel", 10,
    std::bind(&SignalIO::speedCallback, this, std::placeholders::_1));

    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/twist_cmd", 10);


    control_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SignalIO::publishControlLoop, this));

    trajectory_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SignalIO::generateTrajectoryCallback, this));

    startSocketThread();
}

SignalIO::~SignalIO() {
    if (socket_thread_.joinable()) {
        socket_thread_.join();
    }
}

void SignalIO::generateTrajectoryCallback() {
    controller_->generateTrajectory();
}

void SignalIO::gpsCallback(const novatel_oem7_msgs::msg::BESTGNSSPOS::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    if (!gps_ref_set_) {  
        lat0_ = msg->lat;
        lon0_ = msg->lon;
        gps_ref_set_ = true;
        return;
    }
    last_gps_time_ = msg->header.stamp;  // or msg->header.stamp if available
    auto toRadians = [](double deg) { return deg * M_PI / 180.0; };
    auto gps2Distance = [&](double lat1, double lon1, double lat2, double lon2) {
        lat1 = toRadians(lat1); lon1 = toRadians(lon1);
        lat2 = toRadians(lat2); lon2 = toRadians(lon2);
        double dlat = lat2 - lat1, dlon = lon2 - lon1;
        double a = pow(sin(dlat/2),2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2),2);
        return 6371000.0 * 2 * atan2(sqrt(a), sqrt(1-a));
    };

    gps_distance_ = gps2Distance(msg->lat, msg->lon, lat0_, lon0_);
    gps_std_ = msg->lat_stdev + msg->lon_stdev;
}

void SignalIO::imuCallback(const novatel_oem7_msgs::msg::INSPVAX::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    rclcpp::Time current_time = msg->header.stamp;
    if (last_imu_time_.nanoseconds() == 0) {
        last_imu_time_ = current_time;
        return;
    }

    double dt = (current_time - last_imu_time_).seconds();
    double gps_age = (current_time - last_gps_time_).seconds();

    // If GPS is older than 0.3s, consider it stale
    bool gps_fresh = gps_age < 0.3;

    double velocity = std::hypot(msg->north_velocity, msg->east_velocity);
    imu_distance_ += velocity * dt;
    imu_std_ = msg->north_velocity_stdev + msg->east_velocity_stdev;

    last_imu_time_ = current_time;

    if (gps_fresh) {
        double w_gps = imu_std_ / (gps_std_ + imu_std_);
        double w_imu = 1.0 - w_gps;
        double fused_dist = w_gps * gps_distance_ + w_imu * imu_distance_;
        controller_->updatePosition(fused_dist);
    } else {
        controller_->updatePosition(imu_distance_);
    }
}

void SignalIO::speedCallback(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg) {
    double velocity = msg->hor_speed;
    controller_->updateSpeed(velocity);
}

void SignalIO::startSocketThread() {
    socket_thread_ = std::thread(&SignalIO::socketListener, this);
}

void SignalIO::socketListener() {
    int sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Socket creation failed");
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(18080);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_fd, (sockaddr*)&addr, sizeof(addr)) < 0 || listen(sock_fd, 1) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Bind/listen failed");
        close(sock_fd);
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Waiting for client...");
    int client_fd = accept(sock_fd, nullptr, nullptr);
    if (client_fd < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to accept client");
        close(sock_fd);
        return;
    }

    char buffer[1024];
    while (rclcpp::ok()) {
        ssize_t n = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) continue;
        buffer[n] = '\0';

        int id, state;
        int time_left;
        if (sscanf(buffer, "%d,%d,%d", &id, &state, &time_left) == 3) {
            controller_->setTrafficLightCondition(state, time_left);
            RCLCPP_INFO(node_->get_logger(), "Traffic light update: state=%d, time=%d", state, time_left);
            //controller_->generateTrajectory();
        }
    }

    close(client_fd);
    close(sock_fd);
}

void SignalIO::publishControlLoop() {
    static size_t idx = 0;
    const auto& trajectory = controller_->getTrajectory();
    double current_speed = controller_->getLastSpeed();  // if needed, expose a getter

    if (accelerating_to_target_ && current_speed < target_speed_) {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = node_->now();
        cmd.twist.linear.x = std::min(current_speed + 0.2, target_speed_);  // ramp-up speed
        cmd_pub_->publish(cmd);
        return;  // skip normal controller logic
    } else {
        accelerating_to_target_ = false;
    }
    
    if (idx < trajectory.size()) {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = node_->now();
        cmd.twist.linear.x = trajectory[idx].speed;
        cmd_pub_->publish(cmd);
        idx++;
    } else {
        RCLCPP_WARN(node_->get_logger(), "End of trajectory.");
    }
}
