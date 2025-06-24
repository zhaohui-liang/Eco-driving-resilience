#include "rclcpp/rclcpp.hpp"
#include "novatel_oem7_msgs/msg/bestgnsspos.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include <fstream>
#include <chrono>
#include <cmath>

class SignalMonitorNode : public rclcpp::Node
{
public:
    SignalMonitorNode()
        : Node("signal_monitor_node"),
          gps_distance_(0.0),
          gps_ref_set_(false),
          counting_started_(false)
    {
        // Declare and get parameters
        declare_parameter("signal_offset", 0.0);
        declare_parameter("red_duration", 30.0);
        declare_parameter("yellow_duration", 5.0);
        declare_parameter("green_duration", 25.0);
        declare_parameter("disturbance", 0.0);

        signal_offset_ = get_parameter("signal_offset").as_double();
        red_duration_ = get_parameter("red_duration").as_double();
        yellow_duration_ = get_parameter("yellow_duration").as_double();
        green_duration_ = get_parameter("green_duration").as_double();
        disturbance_ = get_parameter("disturbance").as_double();

        start_time_ = now() - rclcpp::Duration::from_seconds(signal_offset_);

        // Subscriptions
        pos_sub_ = create_subscription<novatel_oem7_msgs::msg::BESTGNSSPOS>(
            "/bynav/bestgnsspos", 10,
            std::bind(&SignalMonitorNode::gpsCallback, this, std::placeholders::_1));

        speed_sub_ = create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
            "/bynav/bestvel", 10,
            std::bind(&SignalMonitorNode::speedCallback, this, std::placeholders::_1));

        // Timer for signal update
        signal_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SignalMonitorNode::updateSignalPhase, this));

        // Log file
        log_file_.open("signal_monitor_log.csv");
        log_file_ << "time,lat,lon,speed,phase,phase_time_left\n";
    }

    ~SignalMonitorNode()
    {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTGNSSPOS>::SharedPtr pos_sub_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr speed_sub_;
    rclcpp::TimerBase::SharedPtr signal_timer_;

    rclcpp::Time start_time_;
    double signal_offset_, red_duration_, yellow_duration_, green_duration_, disturbance_;

    int signal_phase_{3};        // 3: Red, 6: Green, 8: Yellow
    int signal_time_left_{0};    // in deciseconds
    double speed_{0.0};
    double gps_distance_;
    bool gps_ref_set_;
    bool counting_started_;
    double lat0_, lon0_;

    std::ofstream log_file_;

    void gpsCallback(const novatel_oem7_msgs::msg::BESTGNSSPOS::SharedPtr msg)
    {
        double current_lat = msg->lat;
        double current_lon = msg->lon;

        if (!counting_started_) return;

        if (!gps_ref_set_) {
            lat0_ = current_lat;
            lon0_ = current_lon;
            gps_ref_set_ = true;
            RCLCPP_INFO(this->get_logger(), "GPS reference set at (%.6f, %.6f)", lat0_, lon0_);
            return;
        }

        auto rad = [](double deg) { return deg * M_PI / 180.0; };
        double dlat = rad(current_lat - lat0_);
        double dlon = rad(current_lon - lon0_);
        double a = pow(sin(dlat / 2), 2) +
                   cos(rad(lat0_)) * cos(rad(current_lat)) * pow(sin(dlon / 2), 2);
        gps_distance_ += 6371000.0 * 2 * atan2(sqrt(a), sqrt(1 - a));

        lat0_ = current_lat;
        lon0_ = current_lon;
        RCLCPP_INFO(this->get_logger(), "Current distance:%.2f m",gps_distance_);
    }

    void speedCallback(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg)
    {
        speed_ = msg->hor_speed;

        if (!counting_started_ && speed_ >= 5.0) {
            counting_started_ = true;
            RCLCPP_INFO(this->get_logger(), "Started counting distance (speed = %.2f m/s)", speed_);
        }
    }

    void updateSignalPhase()
    {
        double t = (now() - start_time_).seconds();
        double cycle = red_duration_ + yellow_duration_ + green_duration_;
        double cycle_time = fmod(t, cycle);

        if (gps_distance_ < 140) {
            if (cycle_time < red_duration_) {
                signal_phase_ = 3;
                signal_time_left_ = static_cast<int>((red_duration_ - cycle_time) * 10);
            } else if (cycle_time < red_duration_ + green_duration_) {
                signal_phase_ = 6;
                signal_time_left_ = static_cast<int>((red_duration_ + green_duration_ - cycle_time) * 10);
            } else {
                signal_phase_ = 8;
                signal_time_left_ = static_cast<int>((cycle - cycle_time) * 10);
            }
        } else {
            int disturbance = static_cast<int>(disturbance_);
            if (cycle_time < red_duration_) {
                signal_phase_ = 3;
                signal_time_left_ = static_cast<int>((red_duration_ - cycle_time) * 10) + disturbance*10;
            } else if (cycle_time < red_duration_ + green_duration_) {
                signal_phase_ = 6;
                signal_time_left_ = static_cast<int>((red_duration_ + green_duration_ - cycle_time) * 10) + disturbance*10;
            } else {
                signal_phase_ = 8;
                signal_time_left_ = static_cast<int>((cycle - cycle_time) * 10) + disturbance*10;
            }
        }

        std::string phase_str;
        std::string color_code;

        if (signal_phase_ == 3) {
            phase_str = "🔴 RED";
            color_code = "\033[1;31m";  // Red
        } else if (signal_phase_ == 6) {
            phase_str = "🟢 GREEN";
            color_code = "\033[1;32m";  // Green
        } else {
            phase_str = "🟡 YELLOW";
            color_code = "\033[1;33m";  // Yellow
        }

        std::string reset_code = "\033[0m";
        RCLCPP_INFO_STREAM(this->get_logger(),
            color_code <<
            "[Traffic Light] Phase: " << phase_str <<
            " | Time left: " << (signal_time_left_ / 10.0) << "s"
            << reset_code);

        if (log_file_.is_open()) {
            log_file_ << now().seconds() << "," << lat0_ << "," << lon0_ << ","
                      << speed_ << "," << signal_phase_ << "," << signal_time_left_/10.0 << "\n";
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
