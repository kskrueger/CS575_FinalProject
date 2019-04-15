//
// Created by Karter Krueger on 2019-04-12.
//

#include "Drivetrain.h"
#include "Lidar.h"

#ifndef INC_575RESEARCHPROJECT_ROBOT_H
#define INC_575RESEARCHPROJECT_ROBOT_H

#endif //INC_575RESEARCHPROJECT_ROBOT_H

//bool RUNNING = false;

class Robot {
public:
    Robot();

    Drivetrain drivetrain = Drivetrain();
    Lidar lidar = Lidar();
};
