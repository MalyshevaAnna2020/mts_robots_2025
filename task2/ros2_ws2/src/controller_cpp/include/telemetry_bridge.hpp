#ifndef TELEMETRY_BRIDGE_HPP
#define TELEMETRY_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
// #include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/static_transform_broadcaster.h>

#include <string>
#include <vector>
#include <memory>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class TelemetryBridge : public rclcpp::Node
{
public:
    explicit TelemetryBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
    void read_telemetry();
    bool recv_all(int sockfd, char* buf, size_t len);

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    
    // TF
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Socket
    int sock_tel_;
    bool is_tcp_;
    std::string tel_host_;
    int tel_port_;

    // LiDAR params (static transform)
    const double LIDAR_X = 0.0;
    const double LIDAR_Y = 0.0;
    const double LIDAR_Z = 0.0;
    const double LIDAR_YAW = 0.0;

    // Pre-allocated scan message
    sensor_msgs::msg::LaserScan scan_msg_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // TELEMETRY_BRIDGE_HPP