//
// Created by Karter Krueger on 2019-04-12.
//
#include "Lidar.h"

using namespace rp::standalone::rplidar;

Lidar::Lidar() {
    drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    checkRPLIDARHealth();
};

void Lidar::start() {
    const char *opt_com_path = NULL;
    _u32 baudrateArray[2] = {115200, 256000};
    _u32 opt_com_baudrate = 0;
    u_result op_result;

    bool useArgcBaudrate = false;

    /*printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: " RPLIDAR_SDK_VERSION "\n");*/

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
        return;
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
        Lidar::on_finished();
    }

    // print out the device serial number, firmware and hardware version number..
    /*printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF,
           (int) devinfo.hardware_version);*/

    // check health...
    if (!Lidar::checkRPLIDARHealth()) {
        Lidar::on_finished();
    }

//    signal(SIGINT, ctrlc);

    drv->startMotor();
    // start scan...
    drv->startScan(0, 1);
}

bool Lidar::checkRPLIDARHealth() {
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

void Lidar::on_finished() {
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
}

void Lidar::lidar_end() {
    drv->stop();
    drv->stopMotor();
    // done!
}
