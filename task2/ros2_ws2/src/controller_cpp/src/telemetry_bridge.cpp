#include "telemetry_bridge.hpp"
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <stdexcept>

TelemetryBridge::TelemetryBridge(const rclcpp::NodeOptions & options)
: Node("telemetry_bridge", options)
{
    // Подключение из примера (на python)
    tel_host_ = this->declare_parameter("TEL_HOST", std::string("0.0.0.0"));
    tel_port_ = this->declare_parameter("TEL_PORT", 5600);
    std::string proto = this->declare_parameter("PROTO", std::string("tcp"));
    is_tcp_ = (proto == "tcp");
    
    sock_tel_ = socket(AF_INET, is_tcp_ ? SOCK_STREAM : SOCK_DGRAM, 0);
    if (sock_tel_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
        throw std::runtime_error("Socket creation failed");
    }

    int enable = 1;
    if (setsockopt(sock_tel_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
        RCLCPP_WARN(this->get_logger(), "setsockopt(SO_REUSEADDR) failed");
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(tel_port_);
    addr.sin_addr.s_addr = inet_addr(tel_host_.c_str());

    if (bind(sock_tel_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Bind failed");
        close(sock_tel_);
        throw std::runtime_error("Bind failed");
    }

    if (is_tcp_) {
        if (listen(sock_tel_, 1) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Listen failed");
            close(sock_tel_);
            throw std::runtime_error("Listen failed");
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for TCP telemetry on %s:%d...", tel_host_.c_str(), tel_port_);
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_sock = accept(sock_tel_, (struct sockaddr*)&client_addr, &client_len);
        if (client_sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Accept failed");
            close(sock_tel_);
            throw std::runtime_error("Accept failed");
        }
        close(sock_tel_); // close listening socket
        sock_tel_ = client_sock;
        RCLCPP_INFO(this->get_logger(), "Connected to telemetry source");
    }

    // Остальной код
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Pre-configure LaserScan
    scan_msg_.header.frame_id = "base_link";
    scan_msg_.angle_min = -M_PI / 4.0;
    scan_msg_.angle_max = M_PI / 4.0;
    scan_msg_.time_increment = 0.0;
    scan_msg_.range_min = 0.2;
    scan_msg_.range_max = 7.5;

    // Timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&TelemetryBridge::read_telemetry, this));
}

void TelemetryBridge::read_telemetry()
{
    try {
        // Перевод примера
        std::vector<uint8_t> buffer;
        if (is_tcp_) {
            uint32_t size_bytes;
            if (recv(sock_tel_, &size_bytes, sizeof(size_bytes), MSG_WAITALL) != sizeof(size_bytes)) {
                return;
            }
            uint32_t size = size_bytes;
            if (size > 65535) return;
            buffer.resize(size);
            if (recv(sock_tel_, buffer.data(), size, MSG_WAITALL) != static_cast<ssize_t>(size)) {
                return;
            }
        } else {
            buffer.resize(65536);
            ssize_t len = recv(sock_tel_, buffer.data(), buffer.size(), 0);
            if (len <= 0) return;
            buffer.resize(len);
        }

        // RCLCPP_INFO(this->get_logger(),
        // "buffer.size() = %ld", buffer.size());

        if (buffer.size() < 4 + 9 * 4 + 4) return;

        size_t offset = 4;
        float odom_x, odom_y, odom_th, vx, vy, vth, wx, wy, wz;
        std::memcpy(&odom_x, buffer.data() + offset, sizeof(float)); offset += 4;
        std::memcpy(&odom_y, buffer.data() + offset, sizeof(float)); offset += 4;
        std::memcpy(&odom_th, buffer.data() + offset, sizeof(float)); offset += 4;
        std::memcpy(&vx, buffer.data() + offset, sizeof(float)); offset += 4;
        std::memcpy(&vy, buffer.data() + offset, sizeof(float)); offset += 4;
        std::memcpy(&vth, buffer.data() + offset, sizeof(float)); offset += 4;
        std::memcpy(&wx, buffer.data() + offset, sizeof(float)); offset += 4;
        std::memcpy(&wy, buffer.data() + offset, sizeof(float)); offset += 4;
        std::memcpy(&wz, buffer.data() + offset, sizeof(float)); offset += 4;

        
        
        uint32_t n;
        std::memcpy(&n, buffer.data() + offset, sizeof(uint32_t)); offset += 4;
        if (n == 0 || buffer.size() < offset + n * 4) return;

        std::vector<float> ranges(n);
        std::memcpy(ranges.data(), buffer.data() + offset, n * sizeof(float));

        // Остальной код

        // RCLCPP_INFO(this->get_logger(),
        // "n = %d", n);

        std::vector<float> filtered_ranges;
        for (float r : ranges) {
            if (r > scan_msg_.range_max || r < scan_msg_.range_min) {
                filtered_ranges.push_back(INFINITY);
            } else {
                filtered_ranges.push_back(r);
            }
        }

        auto now = this->now();

        // === LiDAR ===
        if (!filtered_ranges.empty()) {
            scan_msg_.header.stamp = now;
            scan_msg_.ranges = filtered_ranges;
            scan_msg_.intensities.assign(filtered_ranges.size(), 0.0f);
            scan_msg_.angle_increment = (M_PI / 2.0) / (static_cast<float>(filtered_ranges.size()) - 1.0f);
            scan_pub_->publish(scan_msg_);
        }

        // === TF: odom -> base_link ===
        // geometry_msgs::msg::TransformStamped t;
        // t.header.stamp = now;
        // t.header.frame_id = "odom";
        // t.child_frame_id = "base_link";
        // t.transform.translation.x = odom_x;
        // t.transform.translation.y = odom_y;
        // t.transform.translation.z = 0.0;
        // t.transform.rotation = euler_to_quaternion(0, 0, odom_th);
        // tf_broadcaster_->sendTransform(t);

    } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Parse error: %s", e.what());
    }
}

bool TelemetryBridge::recv_all(int sockfd, char* buf, size_t len)
{
    size_t total = 0;
    while (total < len) {
        ssize_t bytes = recv(sockfd, buf + total, len - total, 0);
        if (bytes <= 0) return false;
        total += bytes;
    }
    return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryBridge>());
  rclcpp::shutdown();
  return 0;
}