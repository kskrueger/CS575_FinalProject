//
// Created by Karter Krueger on 2019-04-12.
//

//***** IMPORTS *****
#include "Drivetrain.h"
#include <chrono>

//***** GLOBALS *****
//bool* RUNNING;

//***** METHODS *****
void resetEncoders(HiTechnicMotor motor1, HiTechnicMotor motor2);

//***** CONSTANTS *****
const int ENCODER_TICKS = 1120;
const double WHEEL_DIAMETER_CM = 9.7; //cm (~4in)
const double RATIO = 28 / 36.0;
const double TICKS_PER_CM = (ENCODER_TICKS / RATIO) / (WHEEL_DIAMETER_CM * 3.14); //encoder count/ticks per cm dist

//***** INITIALIZE *****
HiTechnicDcMotorController mc1(HiTechnicController::DAISY_CHAIN_POSITION_NONE);
HiTechnicMotor motor1(&mc1, HiTechnicDcMotorController::MotorPort::MOTOR_PORT_1);
HiTechnicMotor motor2(&mc1, HiTechnicDcMotorController::MotorPort::MOTOR_PORT_2);

//***** IMU *****
extern "C" {
#include "../BNO055/pi-bno055/include/getbno055.h"
}
char senaddr[256] = "0x28";
struct bnoeul IMU;

Drivetrain::Drivetrain() {
    //RUNNING = running;

    motor1.setDirection(HiTechnicMotor::Direction::FORWARD);
    motor2.setDirection(HiTechnicMotor::Direction::REVERSE);
    motor1.setZeroPowerBehavior(HiTechnicDcMotorController::ZeroPowerBehavior::BRAKE);
    motor2.setZeroPowerBehavior(HiTechnicDcMotorController::ZeroPowerBehavior::BRAKE);

    //resetEncoders(motor1, motor2);

    get_i2cbus(senaddr);
    set_mode(ndof);
};

void wait(int milliseconds) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int elapsed = 0;

    while (elapsed < milliseconds/* && RUNNING*/) {
        std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
        elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(current - begin).count());
    }
}

void Drivetrain::reset_encoders(HiTechnicMotor motor1, HiTechnicMotor motor2) {
    motor1.setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);
    motor2.setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);

    motor1.setRunMode(HiTechnicDcMotorController::RunMode::RUN_TO_POSITION);
    motor2.setRunMode(HiTechnicDcMotorController::RunMode::RUN_TO_POSITION);
}

void Drivetrain::set_motors_powers(int power1, int power2) {
    motor1.setPower(power1);
    motor2.setPower(power2);
}

void Drivetrain::set_motors_powers(int power0) {
    motor1.setPower(power0);
    motor2.setPower(power0);
}

void Drivetrain::time_drive(int power, int milliseconds) {
    set_motors_powers(power);
    wait(milliseconds);
    set_motors_powers(0);
}

void Drivetrain::encoder_drive(int power1, int power2, int target_encoder_counts1, int target_encoder_counts2) {
    motor1.setTargetPosition(target_encoder_counts1);
    motor2.setTargetPosition(target_encoder_counts2);
    set_motors_powers(power1, power2);
    while ((motor1.isBusy() || motor2.isBusy())/* && RUNNING*/) {
        //wait
    }
    set_motors_powers(0);
}

void Drivetrain::encoder_drive(int power, int target_encoder_counts1, int target_encoder_counts2) {
    encoder_drive(power, power, target_encoder_counts1, target_encoder_counts2);
}

void Drivetrain::encoder_drive(int power, int target_encoder_counts) {
    encoder_drive(power, target_encoder_counts, target_encoder_counts);
}

void Drivetrain::distance_drive(int power1, int power2, double distance_cm1, double distance_cm2) {
    encoder_drive(power1, power2, static_cast<int>(distance_cm1 * TICKS_PER_CM),
                  static_cast<int>(distance_cm2 * TICKS_PER_CM));
}

void Drivetrain::distance_drive(int power, double distance_cm) {
    distance_drive(power, power, distance_cm, distance_cm);
}

double Drivetrain::get_heading() {
    get_eul(&IMU);
    return IMU.eul_head;
}
