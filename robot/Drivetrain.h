//
// Created by Karter Krueger on 2019-04-12.
//

#ifndef INC_575RESEARCHPROJECT_DRIVETRAIN_H
#define INC_575RESEARCHPROJECT_DRIVETRAIN_H


#include "../hitec_controller/HiTechnicDcMotorController.h"
#include "../hitec_controller/HiTechnicMotor.h"

class Drivetrain {
public:
    void init(bool *running);

    void reset_encoders(HiTechnicMotor motor1, HiTechnicMotor motor2);

    void set_motors_powers(int power1, int power2);

    void set_motors_powers(int power0);

    void time_drive(int power, int milliseconds);

    void encoder_drive(int power, int target_encoder_counts);

    void encoder_drive(int power, int target_encoder_counts1, int target_encoder_counts2);

    void distance_drive(int power, double distance_cm);

    void distance_drive(int power, double distance_cm1, double distance_cm2);

    void get_heading();


};


#endif //INC_575RESEARCHPROJECT_DRIVETRAIN_H
