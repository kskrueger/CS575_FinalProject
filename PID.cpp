//
// Created by Karter Krueger on 2019-04-19.
//
#include <chrono>
#include <cmath>
#include <cstdio>

class PID {
private:
    double kP;
    double kI;
    double kD;
    std::chrono::steady_clock::time_point lastTime;
    double integral = 0;
    double previousError = 0;
    double out;
    double angleError;
    double error;
    int tolerance = 0;
    bool withinTolerance = false;
    double targetValue;

public:
    PID(double kPa, double kIa, double kDa) {
        kP = kPa;
        kI = kIa;
        kD = kDa;
    }

    PID() {

    }

    void setVariables(double kPa, double kIa, double kDa) {
        kP = kPa;
        kI = kIa;
        kD = kDa;
    }

    void setTolerance(int tolerance1) {
        tolerance = tolerance1;
    }

    int sgn(double x) {
        return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
    }

    double run(double targetValueInput, double position) {
        int dt = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - lastTime).count());
        lastTime = std::chrono::steady_clock::now();
        if (dt < 1) {
            return 0;
        }

        targetValue = targetValueInput;

        if (targetValueInput < 0) {
            targetValue += 360;
        }

        angleError = (targetValue - position);
        angleError -= (360 * floor(0.5 + ((angleError) / 360.0)));

        error = angleError;

        integral += kI * error * dt;

        if (integral > targetValue * 0.25) {
            integral = targetValue * 0.25;
        }

        double u = (kP * error + integral + kD * (error - previousError) / dt);

        previousError = error;
        printf("Turn Error: %f \t Power: %f \n", error, u);

        withinTolerance = fabs(error) < tolerance;

        if (fabs(u) < .05) {
            u += sgn(u) * .05;
        }

        return u;
    }

    double runDistance(double targetValue, double position) {
        int dt = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - lastTime).count());
        lastTime = std::chrono::steady_clock::now();

        if (dt < 1) {
            return 0;
        }

        error = targetValue - position;

        integral += kI * error * dt;

        if (integral > targetValue * 0.25) {
            integral = targetValue * 0.25;
        }

        double u = (kP * error + integral + kD * (error - previousError) / dt);

        printf("Dist Error: %f \t Power: %f \n", error, u);

        previousError = error;

        withinTolerance = fabs(error) < tolerance;

        if (fabs(u) < .05) {
            u += sgn(u) * .05;
        }

        return u;

        /*double error = (targetValue - position);

        integral += kI * error * dt;

        if (integral > targetValue * 0.25) {
            integral = targetValue * 0.25;
        }

        double u = (kP * error + integral + kD * (error - previousError) / dt);

        previousError = error;

        return u;*/
    }

    double getError(/*double targetValue, double position*/) {
        /*double angleError = (targetValue - position);
        angleError -= (360 * floor(0.5 + ((angleError) / 360.0)));*/
        return angleError;
    }
};