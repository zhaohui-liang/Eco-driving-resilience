// signal_io.hpp
#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "vehicle_controller.hpp"
#include "novatel_oem7_msgs/msg/bestgnsspos.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class SignalIO {
public:
    SignalIO(rclcpp::Node* node, std::shared_ptr<VehicleController> controller);
    ~SignalIO();

private:
    void imuCallback(const novatel_oem7_msgs::msg::INSPVAX::SharedPtr msg);
    void gpsCallback(const novatel_oem7_msgs::msg::BESTGNSSPOS::SharedPtr msg);
    void startSocketThread();
    void socketListener();
    void publishControlLoop();
    void generateTrajectoryCallback();

    rclcpp::Node* node_;
    std::shared_ptr<VehicleController> controller_;
    std::thread socket_thread_;

    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVAX>::SharedPtr vel_sub_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTGNSSPOS>::SharedPtr pos_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr trajectory_timer_;

    std::mutex fusion_mutex_;
    bool gps_ref_set_ = false;
    double lat0_ = 0.0, lon0_ = 0.0;
    double gps_distance_ = 0.0, gps_std_ = 1.0;
    double imu_distance_ = 0.0, imu_std_ = 1.0;
    rclcpp::Time last_imu_time_;
    rclcpp::Time last_gps_time_;
};
