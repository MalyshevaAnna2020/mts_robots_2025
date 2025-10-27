#ifndef MAP_BUILDING
#define MAP_BUILDING

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>

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


inline pcl::PointCloud<pcl::PointXYZ>::Ptr lidarToPointCloud(
    const std::vector<double>& lidar,
    double angle_min = -M_PI / 4.0,
    double angle_max = M_PI / 4.0,
    double range_min = 0.3,
    double range_max = 8.0
) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(lidar.size());

    double angle_increment = (angle_max - angle_min) / (lidar.size() - 1);;

    for (size_t i = 0; i < lidar.size(); ++i) {
        double range = lidar[i];
        double angle = angle_min + i * angle_increment;

        if (std::isfinite(range) && range >= range_min && range <= range_max) {
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            double z = 0.0;

            cloud->points.emplace_back(pcl::PointXYZ(x, y, z));
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}

inline std::vector<double> clear_map(double resolution, double min_x, double min_y, double max_x, double max_y){
    int width = static_cast<int>((max_x - min_x) / resolution);
    int height = static_cast<int>((max_y - min_y) / resolution);

    return std::vector<double>(width * height, 0);
}

inline std::vector<double> icp(pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud){
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(current_cloud);
    gicp.setInputTarget(reference_cloud);
    gicp.setMaximumIterations(20);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setMaxCorrespondenceDistance(0.2);

    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    // icp.align(final_cloud);
    // gicp.align(final_cloud);
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    gicp.align(final_cloud, initial_guess);

    // if (icp.hasConverged()) {
    if (gicp.hasConverged()) {
        double fitness = gicp.getFitnessScore();
        std::cout << "___________________________" << std::endl;
        std::cout << "fitness = " << fitness << std::endl;
        std::cout << "___________________________" << std::endl;
        
        // if (fitness > 0.0005) {
        //     // Так отправляется ошибка (1 значение вместо 3)
        //     return {0.0};
        // }

      // Eigen::Matrix4f transform = icp.getFinalTransformation();
      Eigen::Matrix4f transform = gicp.getFinalTransformation();
      // Поворот
      double yaw = atan2(transform(1, 0), transform(0, 0));
      // Смещение
      double dx = transform(0,3);
      double dy = transform(1,3);
      return {dx, dy, yaw};
    }
    else{
        // Возвращаем ошибку
        return {1.0};
    }
}

inline std::vector<int> update_map(std::vector<int> map,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud,
                                      double x, double y, double theta,
                                      double min_x, double max_x,
                                      double min_y, double max_y, double resolution){
    int width = int((max_x - min_x) / resolution);
    for (const auto& point : current_cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;

        double x_new = point.x*cos(theta) - point.y*sin(theta) + x;
        double y_new = point.x*sin(theta) + point.y*cos(theta) + y;

        if (x_new >= min_x && x_new < max_x && y_new >= min_y && y_new < max_y) {
            int cell_x = static_cast<int>((x_new - min_x) / resolution);
            int cell_y = static_cast<int>((y_new - min_y) / resolution);  
            int index = cell_y * width + cell_x;
            map[index] = 100; // занято
        }
    }
    return map;
}

#endif