//
// Created by Karter Krueger on 2019-03-09.
//

#include <iostream>
#include <chrono>
#include <string.h>
#include "hitec_controller/HiTechnicDcMotorController.h"
#include "hitec_controller/HiTechnicMotor.h"
#include "PID.cpp"

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
#include <thread>

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

void gyroTurn(double turnSpeed, double targetAngle, int allowedError, HiTechnicMotor motor1, HiTechnicMotor motor2);

void
distancePID(double driveSpeed, double targetDistCm, double allowedError, HiTechnicMotor motor1, HiTechnicMotor motor2);

void distanceCorrectPID(double driveSpeed, double targetDistCm, double angle, double distError, double angleError,
                        HiTechnicMotor motor1, HiTechnicMotor motor2);

int ROTATION_OFFSET = 180;
const int WORLD_HEIGHT = 5000;
const int WORLD_WIDTH = 5000;

const int ENCODER_TICKS = 1120;
const double WHEEL_DIAMETER_CM = 9.7; //cm
const double RATIO = 28 / 36.0;
const double TICKS_PER_CM = (ENCODER_TICKS / RATIO) / (WHEEL_DIAMETER_CM * 3.14); //encoder count/ticks per cm dist

bool RUNNING = true;

//**************** IMU ****************
extern "C" {
#include "BNO055/pi-bno055/include/getbno055.h"
}
char senaddr[256] = "0x28";
double start_heading = 0;
double heading = 0;

int main(int argc, const char *argv[]) {
    auto imuRead = []() {
        get_i2cbus(senaddr);
        set_mode(ndof);
        struct bnoeul bnod;
        get_eul(&bnod);
        start_heading = bnod.eul_head;
        printf("IMU Thread Start\n");
        while (RUNNING) {
            get_eul(&bnod);
            heading = std::fmod(bnod.eul_head + start_heading, 360);
            //printf("Angle: %f\n", bnod.eul_head);
        }
    };

    // This thread is launched by using
    // lamda expression as callable
    thread imuThread(imuRead);

    HiTechnicDcMotorController mc1(HiTechnicController::DAISY_CHAIN_POSITION_NONE);
    HiTechnicMotor motor1(&mc1, HiTechnicDcMotorController::MotorPort::MOTOR_PORT_1);
    HiTechnicMotor motor2(&mc1, HiTechnicDcMotorController::MotorPort::MOTOR_PORT_2);
    motor2.setDirection(HiTechnicMotor::Direction::REVERSE);
    motor1.setZeroPowerBehavior(HiTechnicDcMotorController::ZeroPowerBehavior::BRAKE);
    motor2.setZeroPowerBehavior(HiTechnicDcMotorController::ZeroPowerBehavior::BRAKE);

    printf("Starting array initialize..... hold on...\n");
    //uint mapArr[WORLD_HEIGHT][WORLD_WIDTH] = {}; // all elements 0
    uint **mapArr = new uint *[WORLD_HEIGHT];
    for (int i = 0; i < WORLD_HEIGHT; i++) {
        mapArr[i] = new uint[WORLD_WIDTH];
    }

    for (int y = 0; y < WORLD_HEIGHT; y++) {
        for (int x = 0; x < WORLD_WIDTH; x++) {
            mapArr[y][x] = 0;
        }
    }

    printf("Array initialized: %d\n", mapArr[0][0]);

    /*rp::standalone::rplidar::RPlidarDriver *drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(
            rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    checkRPLIDARHealth(drv);
    init_lidar(drv);
    u_result op_result;

    double curr_x = WORLD_WIDTH / 2;
    double curr_y = WORLD_HEIGHT / 2;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int elapsed = 0;

    resetEncoders(motor1, motor2);
    double encoder_cm = 0;
    double last_encoders = 0;

    // fetech result and print it out...
    printf("Heading: %f", heading);
    while (*//*encoder_averages / TICKS_PER_CM < 100*//*elapsed < 500) {
        std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
        elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(current - begin).count());

        encoder_cm = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0 / TICKS_PER_CM;
        //cout << "cm travel:" << (encoder_cm) << endl;
        //curr_y = static_cast<int>(WORLD_HEIGHT / 2 + (encoder_averages / TICKS_PER_CM));
        curr_y += (encoder_cm - last_encoders) * sin((heading + 90)* 3.14 / 180);
        curr_x += (encoder_cm - last_encoders) * -cos((heading + 90) * 3.14 / 180);
        last_encoders = encoder_cm;
        printf("Y: %3.2f \t X: %3.2f \t Heading: %3.2f \t Time: %d\n", curr_y, curr_x, heading, elapsed);

        if (ctrl_c_pressed) {
            break;
        }
    }*/


    resetEncoders(motor1, motor2);
    /*distancePID(.8, 100, 2, motor1, motor2);
    gyroTurn(.2, 90, 2, motor1, motor2);
    resetEncoders(motor1, motor2);
    distancePID(.35, 120, 2, motor1, motor2);*/

    distanceCorrectPID(.6, 120, 0, 2, 1, motor1, motor2);


    motor1.setPower(0);
    motor2.setPower(0);

    //lidar_end(drv);

    int firstRow = -1;
    int lastRow = -1;
    int firstCol = -1;
    int lastCol = -1;
    for (int y = 0; y < WORLD_HEIGHT; y++) {
        bool foundValue = false;
        for (int x = 0; x < WORLD_WIDTH; x++) {
            int value = mapArr[y][x];
            if (value > 0) {
                if (firstRow == -1) {
                    firstRow = y;
                }
                if (y > lastRow) {
                    lastRow = y;
                }
                if (firstCol == -1) {
                    firstCol = x;
                }
                if (x > lastCol) {
                    lastCol = x;
                }
            }
        }
    }

    printf("Size: Y:%dx%d, X:%dx%d\n", firstRow, lastRow, firstCol, lastCol);

    if (1) {
        std::ofstream out("roomTest3.csv");
        printf("Writing array to file.... this will be a while....\n");
        for (int y = std::max(firstRow - 2, 0); y < std::min(lastRow + 2, WORLD_HEIGHT); y++) {
            for (int x = std::max(firstCol - 2, 0); x < std::min(lastCol + 2, WORLD_WIDTH); x++) {
                out << mapArr[y][x] << ',';
            }
            out << '\n';
        }
        printf("Write successful!\n");
    }

    printf("Deleting array.... may take a sec....\n");
    for (int i = 0; i < WORLD_HEIGHT; i++) {
        delete[] mapArr[i];
    }
    delete[] mapArr;

    RUNNING = false;
    imuThread.join();
}

void gyroTurn(double turnSpeed, double targetAngle, int allowedError, HiTechnicMotor motor1, HiTechnicMotor motor2) {
    PID turnPID = PID(.1, 0, .08);
    if (targetAngle < 0) {
        targetAngle += 360;
    }
    turnPID.run(targetAngle, heading);
    while (abs(turnPID.getError()) > allowedError && RUNNING) {
        double pidOffset = turnPID.run(targetAngle, heading);
        double power = -pidOffset * turnSpeed;
        motor1.setPower((int8_t) std::min((int) (100 * turnSpeed), (int) (power / 4)));
        motor2.setPower(-(int8_t) std::min((int) (100 * turnSpeed), (int) (power / 4)));
        //printf("Turn: %f\n", heading - targetAngle);
    }
    motor1.setPower(0);
    motor2.setPower(0);
    //robotCentric(0,0,0);
}

void
distancePID(double driveSpeed, double targetDistCm, double allowedError, HiTechnicMotor motor1, HiTechnicMotor motor2) {
    PID distPID = PID(.01, 0, .05);
    double encoders = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0 / TICKS_PER_CM;
    while (abs(targetDistCm - encoders) > allowedError && RUNNING) {
        encoders = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0 / TICKS_PER_CM;
        double pidOffset = distPID.runDistance(targetDistCm, encoders);
        double power = -pidOffset * driveSpeed;
        motor1.setPower((int8_t) std::min((int) (100 * driveSpeed), (int) (power / 4)));
        motor2.setPower((int8_t) std::min((int) (100 * driveSpeed), (int) (power / 4)));
    }
    motor1.setPower(0);
    motor2.setPower(0);
    //robotCentric(0,0,0);
}

void distanceCorrectPID(double driveSpeed, double targetDistCm, double angle, double distError, double angleError,
                        HiTechnicMotor motor1, HiTechnicMotor motor2) {
    PID turnPID = PID(.1, 0, .08);
    const double turnSpeed = driveSpeed / 4;
    if (angle < 0) {
        angle += 360;
    }
    turnPID.run(angle, heading);

    PID distPID = PID(.1, 0, .05);
    double encoders = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0 / TICKS_PER_CM;
    while (abs(targetDistCm - encoders) > distError && RUNNING) {
        encoders = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0 / TICKS_PER_CM;
        double pidOffset = distPID.runDistance(targetDistCm, encoders);
        double power = pidOffset * driveSpeed * 100;
        double turnOffset = turnPID.run(angle, heading);
        double turnPower = turnOffset * turnSpeed * 100;
        double left = power - turnPower;
        double right = power + turnPower;
        double max = std::max(fabs(left), fabs(right));
        left = left / max * 100;
        right = right / max * 100;
        motor1.setPower(static_cast<int8_t>(left));
        motor2.setPower(static_cast<int8_t>(right));
    }
    motor1.setPower(0);
    motor2.setPower(0);
    //robotCentric(0,0,0);
}

void drive_vector(int distance, double angle, HiTechnicMotor motor1, HiTechnicMotor motor2) {
    gyroTurn(.1, angle, 2, motor1, motor2);
}

void follow_vectors(/*std::vector<int, double>*/) {

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