#ifndef TELEMETRY_COMMAND
#define TELEMETRY_COMMAND

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <sstream>

class Telemetry_Command
{
    private:

        // Socket
        int sock_tel;
        bool is_tcp;
        std::string TEL_HOST;
        int TEL_PORT;
        std::string CMD_HOST;
        int CMD_PORT;
        std::string PROTO;
        struct sockaddr_in addr_tel;
        struct sockaddr_in addr_cmd;

        int sock_cmd;

        // Данные лидара
        double range_max = 8;
        double range_min = 0.3;

        // Данные одометрии (которые нам не нужны)
        double odom_x, odom_y, odom_th, vx, vy, vth, wx, wy, wz;

    public:

        std::vector<double> filtered_ranges;

        Telemetry_Command(){
            CMD_HOST = get_env_str("CMD_HOST", "127.0.0.1");
            CMD_PORT = get_env_int("CMD_PORT", 5555);
            TEL_HOST = get_env_str("TEL_HOST", "0.0.0.0");
            TEL_PORT = get_env_int("TEL_PORT", 5600);
            PROTO = get_env_str("PROTO", "tcp");

            // Подключение телеметрии
            is_tcp = (PROTO == "tcp");
            sock_tel = socket(AF_INET, is_tcp ? SOCK_STREAM : SOCK_DGRAM, 0);

            if (sock_tel < 0) {
                perror("Failed to create socket");
                throw std::runtime_error("Failed to create socket");
            }

            int enable = 1;
            if (setsockopt(sock_tel, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
                perror("setsockopt(SO_REUSEADDR) failed");
            }

            memset(&addr_tel, 0, sizeof(addr_tel));
            addr_tel.sin_family = AF_INET;
            addr_tel.sin_port = htons(TEL_PORT);
            addr_tel.sin_addr.s_addr = inet_addr(TEL_HOST.c_str());

            if (bind(sock_tel, (struct sockaddr*)&addr_tel, sizeof(addr_tel)) < 0) {
                perror("Bind failed");
                close(sock_tel);
                throw std::runtime_error("Bind failed");
            }

            // struct timeval tv;
            // tv.tv_sec = 0;
            // tv.tv_usec = 34000; // Сообщения приходят каждые 33.333 мс или с частотой 30 Гц
            // setsockopt(sock_tel, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));


            if (is_tcp) {
                if (listen(sock_tel, 1) < 0) {
                    perror("Listen failed");
                    close(sock_tel);
                    throw std::runtime_error("Listen failed");
                }
                std::cout << "Waiting for TCP telemetry on " << TEL_HOST.c_str() << ":" << TEL_PORT <<"...";
                struct sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);
                int client_sock = accept(sock_tel, (struct sockaddr*)&client_addr, &client_len);
                if (client_sock < 0) {
                    perror("Accept failed");
                    close(sock_tel);
                    throw std::runtime_error("Accept failed");
                }
                close(sock_tel);
                sock_tel = client_sock;
                std::cout << "Connected to telemetry source";
            }

            // Подключение команд
            sock_cmd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock_cmd < 0) {
                perror("Failed to create UDP socket");
                throw std::runtime_error("Failed to create UDP socket");
            }

            std::memset(&addr_cmd, 0, sizeof(addr_cmd));
            addr_cmd.sin_family = AF_INET;
            addr_cmd.sin_port = htons(CMD_PORT);
            if (inet_pton(AF_INET, CMD_HOST.c_str(), &addr_cmd.sin_addr) <= 0) {
                perror("Invalid address / Address not supported");
                close(sock_cmd);
                throw std::runtime_error("Invalid address / Address not supported");
            }
        }

        std::string get_env_str(const char* name, const char* default_val) {
            const char* val = std::getenv(name);
            return val ? std::string(val) : std::string(default_val);
        }

        int get_env_int(const char* name, int default_val) {
            const char* val = std::getenv(name);
            return val ? std::stoi(val) : default_val;
        }

        void read_telemetry(){
            try {
                // Перевод примера
                std::vector<uint8_t> buffer;
                if (is_tcp) {
                    uint32_t size_bytes;
                    if (recv(sock_tel, &size_bytes, sizeof(size_bytes), MSG_WAITALL) != sizeof(size_bytes)) {
                        return;
                    }
                    uint32_t size = size_bytes;
                    if (size > 65535) return;
                    buffer.resize(size);
                    if (recv(sock_tel, buffer.data(), size, MSG_WAITALL) != static_cast<ssize_t>(size)) {
                        return;
                    }
                } else {
                    buffer.resize(65536);
                    ssize_t len = recv(sock_tel, buffer.data(), buffer.size(), 0);
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
                
                filtered_ranges.clear();
                for (double r : ranges) {
                    if (r > range_max) {
                        filtered_ranges.push_back(range_max);
                    } 
                    else if (r < range_min){
                       filtered_ranges.push_back(range_min);
                    } 
                    else {
                        filtered_ranges.push_back(r);
                    }
                }

            } catch (const std::exception & e) {
                std::cout << "Parse error: " << e.what();
            }
        }

        bool recv_all_with_timeout(int sockfd, void* buf, size_t len) {
            size_t total = 0;
            while (total < len) {
                ssize_t bytes = recv(sockfd, static_cast<char*>(buf) + total, len - total, 0);
                if (bytes <= 0) {
                    // Проверим, не таймаут ли
                    if (bytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                        std::cerr << "TCP recv timeout\n";
                    }
                    return false;
                }
                total += bytes;
            }
            return true;
        }

        bool recv_all(int sockfd, char* buf, size_t len)
        {
            size_t total = 0;
            while (total < len) {
                ssize_t bytes = recv(sockfd, buf + total, len - total, 0);
                if (bytes <= 0) return false;
                total += bytes;
            }
            return true;
        }

        void send_cmd(double v, double w) {
            double data[2] = {v, w};
            ssize_t bytes_sent = sendto(
                sock_cmd,
                data,
                sizeof(data),
                0,
                reinterpret_cast<struct sockaddr*>(&addr_cmd),
                sizeof(addr_cmd)
            );

            if (bytes_sent == -1) {
            std::cerr << "sendto() failed: " << strerror(errno) 
                      << " (errno = " << errno << ")" << std::endl;
}

            if (bytes_sent != sizeof(data)) {
                std::cout << "Failed to send full UDP packet" << std::endl;
                std::cout << "bytes_sent = " << bytes_sent << std::endl;
            }
        }

};

#endif