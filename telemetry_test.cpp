//
// Created by Karter Krueger on 2019-04-07.
//

#include <iostream>
#include <chrono>
#include "hitec_controller/HiTechnicDcMotorController.h"
#include "hitec_controller/HiTechnicMotor.h"

using namespace std;

//LIDAR SETUP
bool ctrl_c_pressed;

void ctrlc(int) {
    ctrl_c_pressed = true;
}

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
}
char senaddr[256] = "0x28";

bool RUNNING = true;

HiTechnicDcMotorController mc1(HiTechnicController::DAISY_CHAIN_POSITION_NONE);
HiTechnicMotor motor1(&mc1, HiTechnicDcMotorController::MotorPort::MOTOR_PORT_1);
HiTechnicMotor motor2(&mc1, HiTechnicDcMotorController::MotorPort::MOTOR_PORT_2);

int main(int argc, const char *argv[]) {
    motor2.setDirection(HiTechnicMotor::Direction::REVERSE);
    motor1.setZeroPowerBehavior(HiTechnicDcMotorController::ZeroPowerBehavior::BRAKE);
    motor2.setZeroPowerBehavior(HiTechnicDcMotorController::ZeroPowerBehavior::BRAKE);

    resetEncoders(motor1, motor2);

    int mapArr[WORLD_HEIGHT][WORLD_WIDTH] = {}; // all elements 0

    int curr_y = WORLD_HEIGHT / 2;
    int curr_x = WORLD_WIDTH / 2;

    motor1.setPower(20);
    motor2.setPower(20);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int elapsed = 0;

    // fetech result and print it out...
    while (elapsed < 2000) {
        std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
        elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(current - begin).count());

        //int encoder_averages = (motor1.getCurrentPosition() + motor2.getCurrentPosition())/2;
        cout << "cm travel 1:" << (motor1.getCurrentPosition() / TICKS_PER_CM) << endl;
        cout << "cm travel 2:" << (motor2.getCurrentPosition() / TICKS_PER_CM) << endl;

        if (ctrl_c_pressed) {
            break;
        }
    }
    motor1.setPower(0);
    motor2.setPower(0);

}

void init() {

}

void wait(int milliseconds) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int elapsed = 0;

    while (elapsed < milliseconds && RUNNING) {
        std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
        elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(current - begin).count());
    }
}

void reset_encoders(HiTechnicMotor motor1, HiTechnicMotor motor2) {
    motor1.setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);
    motor2.setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);

    motor1.setRunMode(HiTechnicDcMotorController::RunMode::RUN_TO_POSITION);
    motor2.setRunMode(HiTechnicDcMotorController::RunMode::RUN_TO_POSITION);
}

void set_motors_powers(int power1, int power2) {
    motor1.setPower(power1);
    motor2.setPower(power2);
}

void set_motors_powers(int power0) {
    motor1.setPower(power0);
    motor2.setPower(power0);
}

void time_drive(int power, int milliseconds) {
    set_motors_powers(power);
    wait(milliseconds);
    set_motors_powers(0);
}

void encoder_drive(int power, int target_encoder_counts) {
    motor1.setTargetPosition(target_encoder_counts);
    motor2.setTargetPosition(target_encoder_counts);
    set_motors_powers(power);
    while ((motor1.isBusy() || motor2.isBusy()) && RUNNING) {
        //wait
    }
    set_motors_powers(0);
}

void encoder_drive(int power, int target_encoder_counts1, int target_encoder_counts2) {
    motor1.setTargetPosition(target_encoder_counts1);
    motor2.setTargetPosition(target_encoder_counts2);
    set_motors_powers(power);
    while ((motor1.isBusy() || motor2.isBusy()) && RUNNING) {
        //wait
    }
    set_motors_powers(0);
}

void distance_drive(int power, double distance_cm) {
    encoder_drive(power, static_cast<int>(distance_cm * TICKS_PER_CM));
}

void distance_drive(int power, double distance_cm1, double distance_cm2) {
    encoder_drive(power, static_cast<int>(distance_cm1 * TICKS_PER_CM), static_cast<int>(distance_cm2 * TICKS_PER_CM));
}

void get_heading() {

}
