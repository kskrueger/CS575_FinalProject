//
// Created by Karter Krueger on 2019-04-12.
//

#ifndef INC_575RESEARCHPROJECT_LIDAR_H
#define INC_575RESEARCHPROJECT_LIDAR_H

#include <rplidar.h>
#include <cstdio>
#include <csignal>

class Lidar {
public:
    Lidar();

    rp::standalone::rplidar::RPlidarDriver *drv;

    void start();

    void lidar_end();

    void on_finished();

private:
    bool checkRPLIDARHealth();
};


#endif //INC_575RESEARCHPROJECT_LIDAR_H
