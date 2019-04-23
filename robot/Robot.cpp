//
// Created by Karter Krueger on 2019-04-12.
//

#include "Robot.h"

Robot::Robot() {
    drivetrain = Drivetrain();
    lidar = Lidar();
};

void Robot::stop() {
    drivetrain.imuStop();
}
