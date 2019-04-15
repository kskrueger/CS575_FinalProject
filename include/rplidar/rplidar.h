//
// Created by Karter Krueger on 2019-03-20.
//

#ifndef INC_575RESEARCHPROJECT_RPLIDAR_H
#define INC_575RESEARCHPROJECT_RPLIDAR_H

#include <optional>
#include <ostream>
#include <vector>

// Forward declarations
class rplidar_response_measurement_node_hq_t;
namespace rp {
    namespace standalone {
        namespace rplidar {
            class RPlidarDriver;
        }
    }
}

struct RPLidarScan {
    long int seq;
    std::string frame_id;

    double timestamp;                 // [ns]
    double time_increment;            // [ns]
    double scan_time;                 // [ns]

    double angle_min;                 // [rad]
    double angle_max;                 // [rad]
    double angle_increment;           // [rad]

    double range_min;                 // [m]
    double range_max;                 // [m]

    std::vector<double> ranges;       // [m]
    std::vector<double> intensities;  // [1] (unitless)
};

std::ostream &operator<<(std::ostream &os, const RPLidarScan &scan);

class RPLidar {
private:
    int scan_seq = 0;
    std::string serial_port = "/dev/ttyUSB0";
    int serial_baudrate = 115200;
    std::string scan_mode = "";
    std::string frame_id = "laser_frame";
    double max_distance = 8.0;

    rp::standalone::rplidar::RPlidarDriver *drv = NULL;

public:
    RPLidar(const std::string &serial_port);

    ~RPLidar();

    RPLidar &setBaudrate(int baudrate);

    RPLidar &setFrameID(const std::string &id);

    RPLidar &setMaxDistance(double max_distance);

    RPLidar &setScanMode(const std::string &scan_mode);

    bool getDeviceInfo();

    bool checkHealth();

    bool startMotor();

    bool stopMotor();

    std::optional<RPLidarScan> poll();

private:
    float getAngle(const rplidar_response_measurement_node_hq_t &node);
};

#endif //INC_575RESEARCHPROJECT_RPLIDAR_H
