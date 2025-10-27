#ifndef NEXP_POINT_HPP
#define NEXP_POINT_HPP


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

double LIDAR_STEP = M_PI / 180 / 4;

inline double angles(std::vector<double> lidar){
    // Создание списка направлений
    int d = 20;
    std::vector<double> angle_list;
    int i;

    for (i = d; i < angle_list.size(); i++){
        double al = i * LIDAR_STEP - M_PI / 4;
        double ar = (i - d) * LIDAR_STEP - M_PI / 4;

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

    std::cout << "Список направлений заполнен" << std::endl;

    // углы: "радианы" в [0; 128)
    // все углы < pi/2 по модулю
    // считаем наиболее встречающийся угол
    std::vector<double> dist(128, 0.0);
    for (double a : angle_list){
        int idx = int(a / M_PI * 2 * dist.size()) % dist.size();
        dist[idx] += 1;
    }
    int peak = 0;
    for (i = 0; i < dist.size(); i++) if (dist[i] > dist[peak]) peak = i;

    std::cout << "Индекс максимального значения найден" << std::endl;
    

    int shift_bins = (peak < dist.size() / 2) ? (dist.size() / 2 - peak) : (3 * dist.size() / 2 - peak);
    double shift = double(shift_bins) * M_PI / 2.0 / double(dist.size());

    double total = 0.0;
    double count = 0.0;

    std::cout << "Начало фильтрации" << std::endl;

    // Фильтрация
    for (double a : angle_list){
        double ad = a / M_PI * 2 * dist.size();
        double d1 = int(ad + dist.size() - peak) % dist.size();
        double d2 = int(peak + dist.size() - ad) % dist.size();

        if (d1 > 5 && d2 > 5) continue;

        total += int((a + shift) * 2000000 / M_PI) % 1000000;
        count += 1;
    }

    std::cout << "Конец фильтрации" << std::endl;

    if (count == 0) return 0;

    return (total / count) * M_PI / 2000000 - shift;
}

inline int find_break(std::vector<double> lidar, int direction){
    int start = 80;
    if (direction < 0) start = lidar.size() - start - 1;

    std::cout << "Угол еще не посчитан" << std::endl;
    double diff = angles(lidar) / LIDAR_STEP;
    std::cout << "Угол посчитан" << std::endl;
    std::cout << diff << std::endl;
    
    start -= diff;

    double current = lidar[int(start)];
    if (current > 0.8 && current != INFINITY) {
        std::cout << "current > 0.8: " << current << std::endl;
        return 1;
    }

    int step = abs(direction);
    for (int i = 80; i < 180; i = i + step){
        int idx = int(start);
        if (idx < lidar.size() && lidar[idx] != INFINITY && lidar[idx] - current > 0.35){
            double a = abs(180 - (start + diff)) * LIDAR_STEP;
            std::cout << "a = " << a << std::endl;
            return int((0.25 / tan(a) - 0.15) * 2) + 1;
        }
        current = lidar[idx];
        start += direction;
    }

    return 0;
}

inline int right_break(std::vector<double> lidar){
    std::cout << "right_break" << std::endl;
    return find_break(lidar, 1);
}

inline int left_break(std::vector<double> lidar){
    std::cout << "left_break" << std::endl;
    return find_break(lidar, -1);
}

inline std::vector<double> next_waypoint(std::vector<double> lidar){
    bool rb = right_break(lidar);
    bool lb = left_break(lidar);

    std::cout << "rb = " << rb << std::endl;
    std::cout << "lb = " << lb << std::endl;

    if (rb == 0 && lb == 0) return {0.5, 0.0, 0.0};
    else if (rb > 0) return {0.0, -rb*0.5, -M_PI/2};
    else return {0.0, lb*0.5, M_PI/2};
    
}

#endif