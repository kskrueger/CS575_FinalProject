//
// Created by Karter Krueger on 2019-04-15.
//

#include <hal/util.h>
#include "robot/Robot.h"
#include <cmath>
#include <zconf.h>

bool trackingRunning = true;
int xPosition = 0;
int yPosition = 0;

//***** CONSTANTS *****
const int ENCODER_TICKS = 1120;
const double WHEEL_DIAMETER_CM = 9.7; //cm (~4in)
const double RATIO = 28 / 36.0;
const double TICKS_PER_CM = (ENCODER_TICKS / RATIO) / (WHEEL_DIAMETER_CM * 3.14); //encoder count/ticks per cm dist

//***** METHODS *****
void reset_encoders();

void set_motors_powers(int power1, int power2);

void set_motors_powers(int power0);

void time_drive(int power, int milliseconds);

void encoder_drive(int power1, int power2, int target_encoder_counts1, int target_encoder_counts2);

void encoder_drive(int power, int target_encoder_counts1, int target_encoder_counts2);

void encoder_drive(int power, int target_encoder_counts);

void distance_drive(int power1, int power2, double distance_cm1, double distance_cm2);

void distance_drive(int power, double distance_cm);

HiTechnicMotor *motor1;
HiTechnicMotor *motor2;

std::thread trackingThread;

void startTracking();

void stopTracking();

double getX();

double getY();

Robot *robot2;

int main() {
    Robot robot = Robot();
    robot2 = &robot;
    robot.drivetrain.startThread();
    sleep(10);
    startTracking();
    printf("Heading %4.2lf\n", robot.drivetrain.get_heading());

    HiTechnicDcMotorController mc1(HiTechnicController::DAISY_CHAIN_POSITION_NONE);
    HiTechnicMotor motor1b(&mc1, HiTechnicDcMotorController::MotorPort::MOTOR_PORT_1);
    HiTechnicMotor motor2b(&mc1, HiTechnicDcMotorController::MotorPort::MOTOR_PORT_2);

    motor1 = &motor1b;
    motor2 = &motor2b;

    //motor1->setDirection(HiTechnicMotor::Direction::FORWARD);
    //motor2->setDirection(HiTechnicMotor::Direction::REVERSE);
    motor1->setZeroPowerBehavior(HiTechnicDcMotorController::ZeroPowerBehavior::BRAKE);
    motor2->setZeroPowerBehavior(HiTechnicDcMotorController::ZeroPowerBehavior::BRAKE);

/*    motor1.setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);
    motor2.setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);

    motor1.setRunMode(HiTechnicDcMotorController::RunMode::RUN_USING_ENCODER);
    motor2.setRunMode(HiTechnicDcMotorController::RunMode::RUN_USING_ENCODER);*/
    reset_encoders();

    /*motor1.setPower(20);
    motor2.setPower(20);

    while (motor1.getCurrentPosition() < 1000) {
        printf("Pos: %d\n",motor1.getCurrentPosition());
    }

    motor1.setPower(0);
    motor2.setPower(0);*/

    distance_drive(20, 20);

    printf("Heading2 %4.2lf\n", robot.drivetrain.get_heading());

    robot.stop();
    stopTracking();

    //robot.lidar.start();
    /*while (1) {
        rplidar_response_measurement_node_t nodes[8192];
        size_t count = _countof(nodes);
        u_result op_result = robot.lidar.drv.grabScanData(nodes, count);
        if (IS_OK(op_result)) {
            robot.lidar.drv.ascendScanData(nodes, count);
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
    }*/
    //robot.lidar.lidar_end();
}


void wait(int milliseconds) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int elapsed = 0;

    while (elapsed < milliseconds/* && RUNNING*/) {
        std::chrono::steady_clock::time_point current = std::chrono::steady_clock::now();
        elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(current - begin).count());
    }
}


void reset_encoders() {
    motor1->setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);
    motor2->setRunMode(HiTechnicDcMotorController::RunMode::STOP_AND_RESET_ENCODER);

    motor1->setRunMode(HiTechnicDcMotorController::RunMode::RUN_USING_ENCODER);
    motor2->setRunMode(HiTechnicDcMotorController::RunMode::RUN_USING_ENCODER);
}

void set_motors_powers(int power1, int power2) {
    motor1->setPower(power1);
    motor2->setPower(-power2);
}

void set_motors_powers(int power0) {
    set_motors_powers(power0, power0);
}

void time_drive(int power, int milliseconds) {
    set_motors_powers(power);
    wait(milliseconds);
    set_motors_powers(0);
}

void encoder_drive(int power1, int power2, int target_encoder_counts1, int target_encoder_counts2) {
    //printf("Current pos: 1: %d, 2: %d \t Target pos: 1: %d, 2: %d\n", motor1->getCurrentPosition(), motor2->getCurrentPosition(), target_encoder_counts1, target_encoder_counts2);
    //motor1->setTargetPosition(target_encoder_counts1);
    //motor2->setTargetPosition(target_encoder_counts2);
    set_motors_powers(power1, power2);
    while (abs(motor1->getCurrentPosition()) < abs(target_encoder_counts1) ||
           abs(motor2->getCurrentPosition()) < abs(target_encoder_counts2)) {
        //wait.... sadly this is dead-reckoning now... sigh
        printf("1: %d, 2: %d\n", motor1->getCurrentPosition(), motor2->getCurrentPosition());
    }
    set_motors_powers(0);
}

void encoder_drive(int power, int target_encoder_counts1, int target_encoder_counts2) {
    encoder_drive(power, power, target_encoder_counts1, target_encoder_counts2);
}

void encoder_drive(int power, int target_encoder_counts) {
    encoder_drive(power, target_encoder_counts, target_encoder_counts);
}

void distance_drive(int power1, int power2, double distance_cm1, double distance_cm2) {
    encoder_drive(power1, power2, static_cast<int>(distance_cm1 * TICKS_PER_CM),
                  static_cast<int>(distance_cm2 * TICKS_PER_CM));
}

void distance_drive(int power, double distance_cm) {
    distance_drive(power, power, distance_cm, distance_cm);
}

int prevX = 0;
int prevY = 0;
double prevHeading = 0;
double angleError;
int xPositionAbs = 0;
int yPositionAbs = 0;
double yForwrd;
double xStrafe;

void absolutePosition(int xInput, int yInput, double heading) {
    angleError = (heading - prevHeading);
    angleError -= (360 * floor(0.5 + ((angleError) / 360.0)));

    prevHeading = heading;

    int xDelta = xInput - prevX;
    prevX = xInput;

    int yDelta = yInput - prevY;
    prevY = yInput;

    double strafe = xDelta;// - (xOffset*angleError/360);
    double forwrd = yDelta;// - (yOffset*angleError/360);

    xStrafe += strafe;
    yForwrd += forwrd;

    double gyro_radians = heading * 3.14 / 180;
    double temp = forwrd * cos(gyro_radians) + strafe * sin(gyro_radians);
    strafe = -forwrd * sin(gyro_radians) + strafe * cos(gyro_radians);
    forwrd = temp;

    yPositionAbs += (int) forwrd;
    xPositionAbs += (int) strafe;
}

double getX() {
    return xPositionAbs;
}

double getY() {
    return yPositionAbs;
}

double getAngle() {
    return prevHeading;
}

auto update = []() {
    printf("Inside update thread");
    while (trackingRunning) {
        //double heading = robot2->drivetrain.get_heading();
        int encoderAgv = (motor1->getCurrentPosition() + motor2->getCurrentPosition()) / 2;
        absolutePosition(0, encoderAgv, 0);
        printf("X: %f \t Y: %f \t Heading: %f", getX(), getY(), prevHeading);
    }
};

void startTracking() {
    // This thread is launched by using
    // lamda expression as callable
    trackingThread = std::thread(update);
    printf("update thread started\n");
}

void stopTracking() {
    trackingRunning = false;
    trackingThread.join();
}
