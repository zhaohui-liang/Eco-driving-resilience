#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

class VehicleSender : public rclcpp::Node {
public:
  VehicleSender() : Node("vehicle_sender") {
    using std::placeholders::_1;

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/twist_cmd", 10, std::bind(&VehicleSender::twistCallback, this, _1));

    std::thread(&VehicleSender::socketServer, this).detach();
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  std::mutex cmd_mutex_;
  double linear_x_ = 0.0;
  double angular_z_ = 0.0;

  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    linear_x_ = msg->twist.linear.x;
    angular_z_ = msg->twist.angular.z;
  }

  void socketServer() {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
      perror("socket");
      return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(10001);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
      perror("bind");
      close(sock);
      return;
    }

    if (listen(sock, 20) == -1) {
      perror("listen");
      close(sock);
      return;
    }

    while (rclcpp::ok()) {
      sockaddr_in client;
      socklen_t len = sizeof(client);
      int client_sock = accept(sock, (struct sockaddr*)&client, &len);
      if (client_sock == -1) {
        perror("accept");
        continue;
      }

      double linear_x, angular_z;
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        linear_x = linear_x_;
        angular_z = angular_z_;
      }

      std::ostringstream oss;
      oss << linear_x << "," << angular_z << ";";

      std::string cmd = oss.str();
      ssize_t n = write(client_sock, cmd.c_str(), cmd.size());
      if (n < 0) perror("write");
      if (close(client_sock) == -1) perror("close");

      std::cout << "Sent: " << cmd << " (" << cmd.size() << " bytes)" << std::endl;
    }

    close(sock);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleSender>());
  rclcpp::shutdown();
  return 0;
}
