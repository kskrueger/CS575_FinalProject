//
// Created by Karter Krueger on 2019-04-15.
//

#include <hal/util.h>
#include "robot/Robot.h"

int main() {
    Robot robot = Robot();

    while (1) {
        printf("%4.2lf\n", robot.drivetrain.get_heading());
    }

    /*robot.lidar.start();
    while (1) {
        rplidar_response_measurement_node_t nodes[8192];
        size_t count = _countof(nodes);
        u_result op_result = robot.lidar.drv->grabScanData(nodes, count);
        if (IS_OK(op_result)) {
            robot.lidar.drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int) count; ++pos) {
                //nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT;
                if ((nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) == 47) {
                    double theta = ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f) * 3.14 / 180;
                    float dist = nodes[pos].distance_q2 / 4.0f;
                    int x_dist = static_cast<int>(dist * cos(theta) / 10);
                    int y_dist = static_cast<int>(dist * -sin(theta) / 10);
                    //printf("X: %d \t Y: %d", x_dist, y_dist);
                    //printf("%03.2f \t %08.2f \n", theta, dist);
                    printf("%d \t %d \n", x_dist, y_dist);
                };
            }
        }
    }
    robot.lidar.lidar_end();*/
}
