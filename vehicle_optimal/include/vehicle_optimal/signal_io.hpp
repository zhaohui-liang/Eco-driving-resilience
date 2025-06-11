// signal_io_internal.hpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <novatel_oem7_msgs/msg/inspvax.hpp>
#include <novatel_oem7_msgs/msg/bestgnsspos.hpp>
#include <novatel_oem7_msgs/msg/bestvel.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include "vehicle_optimal/vehicle_optimal.hpp"

class SignalIO {
public:
  SignalIO(rclcpp::Node* node, std::shared_ptr<VehicleController> controller);

private:
  void imuCallback(const novatel_oem7_msgs::msg::INSPVAX::SharedPtr msg);
  void gpsCallback(const novatel_oem7_msgs::msg::BESTGNSSPOS::SharedPtr msg);
  void speedCallback(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg);
  void generateTrajectoryCallback();
  void publishControlLoop();
  void updateSignalPhase();

  rclcpp::Node* node_;
  std::shared_ptr<VehicleController> controller_;

  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTGNSSPOS>::SharedPtr pos_sub_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr speed_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr trajectory_timer_;
  rclcpp::TimerBase::SharedPtr signal_timer_;

//   rclcpp::Time last_imu_time_;
  rclcpp::Time last_gps_time_;

  double lat0_ = 0.0;
  double lon0_ = 0.0;
  bool gps_ref_set_ = false;

  double gps_distance_ = 0.0;
//   double imu_distance_ = 0.0;
  double gps_std_ = 0.0;
//   double imu_std_ = 1.0;

  std::mutex fusion_mutex_;

  bool accelerating_to_target_ = true;
  double entry_speed_;

  rclcpp::Time start_time_;
  int signal_phase_ = 0;
  int signal_time_left_ = 0;
  double red_duration_;
  double green_duration_;
  double yellow_duration_;
  double signal_offset;
};

