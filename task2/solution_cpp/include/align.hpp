#ifndef ALIGN
#define ALIGN

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

double LIDAR_STEP_ALIGN = M_PI / 180 / 4;

inline double align(std::vector<double> lidar){
    // Создание списка направлений
    int d = 20;
    std::vector<double> angle_list;
    int i;

    for (i = d; i < angle_list.size(); i++){
        double al = i * LIDAR_STEP_ALIGN - M_PI / 4;
        double ar = (i - d) * LIDAR_STEP_ALIGN - M_PI / 4;

        double xl = lidar[i] * sin(al);
        double yl = lidar[i] * cos(al);
        double xr = lidar[i - d] * sin(ar);
        double yr = lidar[i - d] * cos(ar);
        double dy = yr - yl;
        double dx = xr - xl;

        double a = atan2(dy, dx);
        if (a < 0) a += 2*M_PI;
        if (dy == 0 && dx == 0) a = 0.0;

        angle_list.push_back(a);
    }

    double angle_mid = angle_list[int(angle_list.size() / 2)];
    double sum_angle = 0;
    int count_angle = 0;
    for (int i = angle_list.size() / 2; abs(angle_list[i] - angle_mid) < 5 * M_PI / 180; ++i){
        sum_angle += sum_angle;
        count_angle++;
    }
    for (int i = angle_list.size() / 2; abs(angle_list[i] - angle_mid) < 5 * M_PI / 180; --i){
        sum_angle += sum_angle;
        count_angle++;
    }

    return -sum_angle/count_angle;
}

#endif