// signal_io_internal.cpp
#include "vehicle_controller/signal_io.hpp"
#include <cmath>
#include <mutex>

SignalIO::SignalIO(rclcpp::Node* node, std::shared_ptr<VehicleController> controller)
: node_(node), 
  controller_(controller)
{
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

    node_->declare_parameter("signal_offset", 0.0);
    signal_offset = node_->get_parameter("signal_offset").as_double();
    start_time_ = node_->now() - rclcpp::Duration::from_seconds(signal_offset);

    red_duration_ = node_->get_parameter("red_duration").as_double();
    yellow_duration_ = node_->get_parameter("yellow_duration").as_double();
    green_duration_ = node_->get_parameter("green_duration").as_double();

    node_->declare_parameter("entry_speed", 0.0);
    entry_speed_ = node->get_parameter("entry_speed").as_double();

    signal_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SignalIO::updateSignalPhase, this));
}


void SignalIO::generateTrajectoryCallback() {
    if (!accelerating_to_target_) {
        controller_->generateTrajectory();
    }
}

void SignalIO::gpsCallback(const novatel_oem7_msgs::msg::BESTGNSSPOS::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (accelerating_to_target_) {
        return;  // don't set or update anything during acceleration
    }

    if (!gps_ref_set_) {  
        lat0_ = msg->lat;
        lon0_ = msg->lon;
        gps_ref_set_ = true;
        return;
    }
    last_gps_time_ = msg->header.stamp;
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
    controller_->updatePosition(gps_distance_);
}


void SignalIO::speedCallback(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg) {
    double velocity = msg->hor_speed;
    controller_->updateSpeed(velocity);
}

void SignalIO::updateSignalPhase() {
    rclcpp::Duration elapsed = node_->now() - start_time_;
    double t = elapsed.seconds();
    // RCLCPP_INFO(node_->get_logger(), "elapsed time: %.2f s", t);

    double cycle = red_duration_ + yellow_duration_ + green_duration_;
    double cycle_time = fmod(t, cycle);

    if (cycle_time < red_duration_) {
        signal_phase_ = 3;  // Red
        signal_time_left_ = static_cast<int>((red_duration_ - cycle_time) * 10);
    } else if (cycle_time < red_duration_ + green_duration_) {
        signal_phase_ = 6;  // Green
        signal_time_left_ = static_cast<int>((red_duration_ + green_duration_ - cycle_time) * 10);
    } else {
        signal_phase_ = 8;  // Yellow
        signal_time_left_ = static_cast<int>((cycle - cycle_time) * 10);
    }

    controller_->setTrafficLightCondition(signal_phase_, signal_time_left_);
}

void SignalIO::publishControlLoop() {
    static size_t idx = 0;
    const auto& trajectory = controller_->getTrajectory();
    double current_speed = controller_->getLastSpeed();
    RCLCPP_WARN(node_->get_logger(), "accelerating_to_target_:%d",accelerating_to_target_);

    if (accelerating_to_target_ && current_speed < entry_speed_) {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = node_->now();
        cmd.twist.linear.x = std::min(current_speed + 1.0, entry_speed_);
        cmd_pub_->publish(cmd);
        return;
    } else {
        accelerating_to_target_ = false;
        gps_distance_ = 0.0;  // start clean
        controller_->generateTrajectory();
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
