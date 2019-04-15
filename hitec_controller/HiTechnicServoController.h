/*
 * Copyright (c) 2019 FTC team 4634 FROGbots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "HiTechnicController.h"

#ifndef HiTechnicServoController_h
#define HiTechnicServoController_h

class HiTechnicServoController : public HiTechnicController {
public:
    HiTechnicServoController(DaisyChainPosition pos);

    enum ServoPort {
        SERVO_PORT_1 = 0, SERVO_PORT_2, SERVO_PORT_3, SERVO_PORT_4, SERVO_PORT_5, SERVO_PORT_6
    };

    void setServoPosition(ServoPort port, uint8_t pos);

    void enablePwm(bool noTimeout);

    void disablePwm();

    void setStepTime(int8_t t);

    uint8_t getServoPosition(ServoPort port);

private:
    uint8_t commandedPositions[6] = {0, 0, 0, 0, 0, 0};

    static const uint8_t REGISTER_SERVO_1_POS = 0x42; //servo 6 reg -> 0x47
    static const uint8_t REGISTER_ENABLE_PWM = 0x48;
    static const uint8_t REGISTER_STEP_TIME = 0x41;
    static const uint8_t PWM_DISABLED = 255;
    static const uint8_t PWM_ENABLED = 0;
    static const uint8_t PWM_ENABLED_NTO = 0xAA;
};

#endif
