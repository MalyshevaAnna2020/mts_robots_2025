#include "../include/tel_cmd.hpp"
#include "../include/nexp_point.hpp"
#include "../include/map_building.hpp"
#include "../include/align.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>

#include <thread>
#include <chrono>

int main(int argc, char * argv[])
{
    Telemetry_Command tel;
    std::vector<double> map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    bool first_scan_received = false;

    std::vector<double> lidar;

    // Данные посылки скорости
    double v = 0.0;
    double w = 0.0;

    // Относительное положение робота
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    // Параметры карты
    double x_min = -8.0;
    double x_max = 8.0;
    double y_min = -8.0;
    double y_max = 8.0;
    double resolution = 0.025;

    // Цель
    double x_goal = 0.0;
    double y_goal = 0.0;
    // double theta_goal = 0.0;

    // Параметры ПИД-регулятора
    double K_P_V = 100;
    double K_P_W = 0.01;

    double dist;

    // === Основной цикл ===
    while(true){
        // Начальное положение
        x = x - x_goal;
        y = y - y_goal;
        theta = 0.0;
        // theta = theta - theta_goal;

        std::cout << "Текущее положение:" << std::endl;
        std::cout << "(" << x << "; " << y << "; " << theta << ")" << std::endl;

        tel.read_telemetry();
        std::cout << "Данные лидара получены" << std::endl;
        // std::cout << "Данные лидара получены:" << std::endl << "(";
        // for (double range : tel.filtered_ranges){
        //     std::cout << range << "; ";
        // }
        // std::cout << ")" << std::endl;

        // Цель
        std::vector<double> point = next_waypoint(tel.filtered_ranges);
        double dtheta_goal = point[2];
        x_goal = point[0];
        y_goal = point[1];
        
        std::cout << "Следующая цель:" << std::endl;
        std::cout << "(" << x_goal << "; " << y_goal << "; " << dtheta_goal << ")" << std::endl;

        // A* algorithm (получение пути здесь и следование по точкам в цикле while)
        // или обычный ПИД-регулятор (в следующем цикле while) - что и сделано

        // map = clear_map(resolution, x_min, y_min, x_max, y_min);

        dist = sqrt((x_goal - x)*(x_goal - x) + (y_goal - y)*(y_goal - y));
        std::cout << "Расстояние до цели: " << dist << std::endl;

        // simultanious read_tel & building_map & localization & navigation & send_vel
        // Доехать до точки
        while(dist > 0.1){
            // Получение данных из webots
            tel.read_telemetry();

            // localization
            current_cloud = lidarToPointCloud(tel.filtered_ranges);

            // Первый скан
            if (!first_scan_received) {
                *reference_cloud = *current_cloud;
                first_scan_received = true;
                std::cout << "Первый скан сохранён как опорный." << std::endl;
                continue;
            }
            // Смещение
            std::vector<double> delta = icp(current_cloud, reference_cloud);
            if (delta.size() == 1) continue;
            double dx = delta[0];
            double dy = delta[1];
            double dtheta = delta[2];
            // Новые координаты
            theta = theta + dtheta;
            x = x + dx*cos(theta) - dy*sin(theta);
            y = y + dx*sin(theta) + dy*cos(theta);

            std::cout << "Текущее положение:" << std::endl;
            std::cout << "(" << x << "; " << y << "; " << theta << ")" << std::endl;

            // update_map
            // map = update_map(map, current_cloud, x, y, theta, resolution)

            // navigation
            dist = sqrt((x_goal - x)*(x_goal - x) + (y_goal - y)*(y_goal - y));
            double dist_control = dist;
            double angle = dtheta_goal - theta;
            if (abs(angle) > 0.5) {
                // auto now_tp = std::chrono::steady_clock::now();
                // double now = std::chrono::duration<double>(now_tp.time_since_epoch()).count();
                dist_control = 0.1;
            }
            if (abs(angle) < 0.1){
                angle = 0.0;
            }
            v = K_P_V*dist_control;
            w = K_P_W*angle;

            if (w > 1){
                w = 1;
            }
            if (w < -1){
                w = -1;
            }

            std::cout << "dist = " << dist << "; angle = " << angle << std::endl;
            std::cout << "v = " << v << "; w = " << w << std::endl;

            // Отправка скоростей в webots
            tel.send_cmd(v, w);

        }
        // // Поворот
        // while (abs(theta_goal - theta) < 5*M_PI/180){
        //     v = 0;
        //     w = K_P_W*(theta_goal - theta);
        //     tel.send_cmd(v, w);

        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        // }
        
        // // align
        // tel.read_telemetry();
        // double angle_align = align(tel.filtered_ranges);

        // while(abs(angle_align) > 1*M_PI/180){            
        //     v = 0;
        //     w = K_P_W*angle_align;
        //     tel.send_cmd(v, w);

        //     tel.read_telemetry();
        //     angle_align = align(tel.filtered_ranges);

        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        // } 
    }
}