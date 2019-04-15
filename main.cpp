//
// Created by Karter Krueger on 2019-03-09.
//

#include <iostream>
#include <chrono>
#include <string.h>
#include "hitec_controller/HiTechnicDcMotorController.h"
#include "hitec_controller/HiTechnicMotor.h"

#include <math.h>
#include <fstream>

using namespace std;

//LIDAR SETUP
#include <stdlib.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#include <unistd.h>

static inline void delay(_word_size_t ms) {
    while (ms >= 1000) {
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0)
        usleep(ms * 1000);
}

#include <signal.h>

bool ctrl_c_pressed;

void ctrlc(int) {
    ctrl_c_pressed = true;
}

bool checkRPLIDARHealth(rp::standalone::rplidar::RPlidarDriver *drv);

void init_lidar(rp::standalone::rplidar::RPlidarDriver *drv);

void lidar_end(rp::standalone::rplidar::RPlidarDriver *drv);

void on_finished(rp::standalone::rplidar::RPlidarDriver *drv);

//**************** MOTORS AND ENCODERS ****************
void resetEncoders(HiTechnicMotor motor1, HiTechnicMotor motor2);

int ROTATION_OFFSET = 180;
const int WORLD_HEIGHT = 2000;
const int WORLD_WIDTH = 500;

const int ENCODER_TICKS = 1120;
const double WHEEL_DIAMETER_CM = 9.7; //cm
const double RATIO = 28 / 36.0;
const double TICKS_PER_CM = (ENCODER_TICKS / RATIO) / (WHEEL_DIAMETER_CM * 3.14); //encoder count/ticks per cm dist

//**************** IMU ****************
extern "C" {
#include "BNO055/pi-bno055/include/getbno055.h"
}
char senaddr[256] = "0x28";

int main(int argc, const char *argv[]) {
    get_i2cbus(senaddr);
    set_mode(ndof);
    struct bnoeul bnod;

    HiTechnicDcMotorController mc1(HiTechnicController::DAISY_CHAIN_POSITION_NONE);
    HiTechnicMotor motor1(&mc1, HiTechnicDcMotorController::MotorPort::MOTOR_PORT_1);
    HiTechnicMotor motor2(&mc1, HiTechnicDcMotorController::MotorPort::MOTOR_PORT_2);
    motor2.setDirection(HiTechnicMotor::Direction::REVERSE);
    motor1.setZeroPowerBehavior(HiTechnicDcMotorController::ZeroPowerBehavior::BRAKE);
    motor2.setZeroPowerBehavior(HiTechnicDcMotorController::ZeroPowerBehavior::BRAKE);

    resetEncoders(motor1, motor2);
    uint mapArr[WORLD_HEIGHT][WORLD_WIDTH] = {}; // all elements 0

    rp::standalone::rplidar::RPlidarDriver *drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(
            rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    checkRPLIDARHealth(drv);
    init_lidar(drv);
    u_result op_result;

    int curr_x = WORLD_WIDTH / 2;
    int curr_y = WORLD_HEIGHT / 2;

    motor1.setPower(25);
    motor2.setPower(25);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int elapsed = 0;

    double angle;
    int encoder_averages;

    // fetech result and print it out...
    while (encoder_averages / TICKS_PER_CM < 750) {
        std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
        elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(current - begin).count());

        encoder_averages = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2;
        cout << "cm travel:" << (encoder_averages / TICKS_PER_CM) << endl;
        curr_y = static_cast<int>(WORLD_HEIGHT / 2 + (encoder_averages / TICKS_PER_CM));

        rplidar_response_measurement_node_t nodes[8192];
        size_t count = _countof(nodes);
        op_result = drv->grabScanData(nodes, count);
        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int) count; ++pos) {
                //nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT;
                if ((nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) == 47) {
                    get_eul(&bnod);
                    angle = 0 <= bnod.eul_head && bnod.eul_head <= 360 ? bnod.eul_head : angle;
                    double theta = ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f +
                                    ROTATION_OFFSET + angle) * 3.14 / 180;
                    float dist = nodes[pos].distance_q2 / 4.0f;
                    int x_dist = curr_x + static_cast<int>(dist * cos(theta) / 10);
                    int y_dist = curr_y + static_cast<int>(dist * -sin(theta) / 10);
                    if (x_dist > 0 && x_dist < WORLD_WIDTH && y_dist > 0 && y_dist < WORLD_HEIGHT) {
                        if (mapArr[y_dist][x_dist] <= 1000) {
                            mapArr[y_dist][x_dist] += 1;
                        }
                    }
                    printf("Angle: %f", angle);
                    //printf("%03.2f \t %08.2f \n", theta, dist);
                    //printf("%d \t %d \n", x_dist, y_dist);
                };
            }
        }

        if (ctrl_c_pressed) {
            break;
        }
    }

    motor1.setPower(0);
    motor2.setPower(0);

    lidar_end(drv);

    std::ofstream out("test3.csv");

    for (auto &row : mapArr) {
        for (auto col : row) {
            out << col << ',';
        }
        out << '\n';
    }
}

void resetEncoders(HiTechnicMotor motor1, HiTechnicMotor motor2) {
    motor1.setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);
    motor2.setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);

    motor1.setRunMode(HiTechnicDcMotorController::RunMode::RUN_USING_ENCODER);
    motor2.setRunMode(HiTechnicDcMotorController::RunMode::RUN_USING_ENCODER);
}

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver *drv) {
    rplidar_response_device_health_t healthinfo;
    u_result op_result;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

void init_lidar(rp::standalone::rplidar::RPlidarDriver *drv) {
    const char *opt_com_path = NULL;
    _u32 baudrateArray[2] = {115200, 256000};
    _u32 opt_com_baudrate = 0;
    u_result op_result;

    bool useArgcBaudrate = false;

    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: " RPLIDAR_SDK_VERSION "\n");

    // read serial port from the command line...
/*    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3"

    // read baud rate from the command line if specified...
    if (argc>2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }*/

    if (!opt_com_path) {
        opt_com_path = "/dev/ttyUSB0";
    }

    // create the driver instance
    if (!drv) {
        fprintf(stderr, "insufficient memory, exit\n");
        exit(-2);
    }

    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if (useArgcBaudrate) {
        if (!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate))) {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result)) {
                connectSuccess = true;
            } else {
                delete drv;
                drv = NULL;
            }
        }
    } else {
        size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
        for (size_t i = 0; i < baudRateArraySize; ++i) {
            if (!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if (IS_OK(drv->connect(opt_com_path, baudrateArray[i]))) {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result)) {
                    connectSuccess = true;
                    break;
                } else {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {

        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_com_path);
        on_finished(drv);
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF,
           (int) devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        on_finished(drv);
    }

    signal(SIGINT, ctrlc);

    drv->startMotor();
    // start scan...
    drv->startScan(0, 1);
}

void lidar_end(RPlidarDriver *drv) {
    drv->stop();
    drv->stopMotor();
    // done!
}

void on_finished(RPlidarDriver *drv) {
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
}

/*someMotor2.setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);
    someMotor2.setRunMode(HiTechnicDcMotorController::RunMode::RUN_USING_ENCODER);
    someMotor2.setDirection(HiTechnicMotor::Direction::FORWARD);
    someMotor2.setPower(20);
    someMotor2.getController()->setTimeoutEnabled(false);*/
//someMotor.setRunMode(HiTechnicDcMotorController::RunMode::RUN_USING_ENCODER);
/*while (someMotor.getCurrentPosition() > 2000) {
    someMotor.setPower(30);
    someMotor2.setPower(30);
    cout<<someMotor.getCurrentPosition()<<"\n";
}*/
/*while (someMotor.getCurrentPosition() > 0) {
    someMotor.setPower(-30);
    someMotor2.setPower(-30);
    cout<<someMotor.getCurrentPosition()<<"\n";
}*/
/*while (elapsed < 1000 && i < height) {
    std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
    elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(current - begin).count());
    //std::cout << "Time difference = " << elapsed <<std::endl;
    dataArr[++i][0] = elapsed;
    dataArr[i][1] = motor1.getCurrentPosition();
    dataArr[i][2] = motor2.getCurrentPosition();
    dataArr[i][3] = 0;
}

for (int y = 0; y < i; y++) {
    for (int x = 0; x < width; x++) {
        cout<<std::to_string(dataArr[y][x])<<'\t';
    }
    cout<<endl;
}*/