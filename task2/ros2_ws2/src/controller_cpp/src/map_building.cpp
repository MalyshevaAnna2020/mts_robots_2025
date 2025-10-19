#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry/laser_geometry.hpp>

#include <pcl/filters/voxel_grid.h>

#include "telemetry_bridge.hpp"
#include "utils.hpp"

using namespace std::chrono_literals;

class MapLidar : public rclcpp::Node
{
public:
  MapLidar()
  : Node("map_lidar")
  {
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&MapLidar::scan_callback, this, std::placeholders::_1)
    );
    occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&MapLidar::timer_callback, this));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    reference_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    current_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    first_scan_received_ = false;

    x = 0.0;
    y = 0.0;
    theta = 0.0;

    // Карта
    grid.header.stamp = rclcpp::Clock().now();
    grid.header.frame_id = "odom";
    resolution = 0.025;
    min_x = -1;
    min_y = -1;
    max_x = 9;
    max_y = 9;
    width = static_cast<int>((max_x - min_x) / resolution);
    height = static_cast<int>((max_y - min_y) / resolution);
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = min_x;
    grid.info.origin.position.y = min_y;
    grid.info.origin.orientation.w = 1.0;

    //  -1 - неизвестно
    //   0 - свободно
    // 100 - занято (препятствие)
    grid.data.resize(width * height, 0);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  laser_geometry::LaserProjection projector_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
  bool first_scan_received_;
  nav_msgs::msg::OccupancyGrid grid;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  double resolution, min_x, min_y, max_x, max_y;
  int32_t width, height;

  // Смещение текущей карты относительно самой первой
  double x, y, theta;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    // Преобразуем LaserScan в PointCloud2
    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    projector_.projectLaser(*scan_msg, point_cloud_msg);

    // Преобразуем в PCL облако
    pcl::fromROSMsg(point_cloud_msg, *current_cloud);

    // Первый скан
    if (!first_scan_received_) {
      *reference_cloud_ = *current_cloud;
      first_scan_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Первый скан сохранён как опорный.");
      return;
    }
  }

  void timer_callback()
  {
    // // Настройка ICP
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(current_cloud);
    // icp.setInputTarget(reference_cloud_);
    // icp.setMaximumIterations(20);
    // icp.setTransformationEpsilon(1e-10);
    // icp.setEuclideanFitnessEpsilon(1e-10);
    // icp.setMaxCorrespondenceDistance(0.05);
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(current_cloud);
    gicp.setInputTarget(reference_cloud_);
    gicp.setMaximumIterations(20);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setMaxCorrespondenceDistance(0.5);

    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    // icp.align(final_cloud);
    // gicp.align(final_cloud);
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    gicp.align(final_cloud, initial_guess);

    // if (icp.hasConverged()) {
    if (gicp.hasConverged()) {
      // Eigen::Matrix4f transform = icp.getFinalTransformation();
      Eigen::Matrix4f transform = gicp.getFinalTransformation();
      // Поворот
      double yaw = atan2(transform(1, 0), transform(0, 0));
      theta = theta + yaw;
      // Смещение
      double dx = transform(0,3);
      double dy = transform(1,3);
      x = x + dx*cos(theta) - dy*sin(theta);
      y = y + dx*sin(theta) + dy*cos(theta);

      RCLCPP_INFO(this->get_logger(),
        "Координаты: (%.3f, %.3f) м, Поворот: %.2f°",
        x, y, theta * 180.0 / M_PI);
    } else {
      RCLCPP_WARN(this->get_logger(), "ICP не сошёлся.");
      return;
    }

    *reference_cloud_ = *current_cloud;

    // Обновление карты
    // Заполняем занятые ячейки
    for (const auto& point : current_cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;

        // Пока карта смещается вверх-вниз и вправо-влево
        double x_new = point.x*cos(theta) - point.y*sin(theta) + x;
        double y_new = point.x*sin(theta) + point.y*cos(theta) + y;

        // RCLCPP_INFO(this->get_logger(),
        //       "cell_x = %d", cell_x);

        if (x_new >= min_x && x_new < max_x && y_new >= min_y && y_new < max_y) {
            int cell_x = static_cast<int>((x_new - min_x) / resolution);
            int cell_y = static_cast<int>((y_new - min_y) / resolution);  
            int index = cell_y * width + cell_x;
            grid.data[index] = 100; // занято
        }
    }
    grid.header.stamp = rclcpp::Clock().now();

    occupancy_grid_publisher_->publish(std::move(grid));

    // === TF: odom -> base_link ===
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = rclcpp::Clock().now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = euler_to_quaternion(0, 0, theta);
    tf_broadcaster_->sendTransform(t);
  }

  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapLidar>());
  rclcpp::shutdown();
  return 0;
}