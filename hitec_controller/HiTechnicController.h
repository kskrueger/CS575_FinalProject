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


#ifndef HiTechnicController_h
#define HiTechnicController_h

#include <cstdint>

class HiTechnicController {
public:
    enum DaisyChainPosition {
        DAISY_CHAIN_POSITION_FIRST = 0x01,
        DAISY_CHAIN_POSITION_SECOND,
        DAISY_CHAIN_POSITION_THIRD,
        DAISY_CHAIN_POSITION_FOURTH,
        DAISY_CHAIN_POSITION_NONE
    };

    HiTechnicController(DaisyChainPosition pos);

    void getManufacturer(uint8_t *out);

    void getSensorType(uint8_t *out);

protected:
    void write8(uint8_t reg, uint8_t val);

    void writeSigned32(uint8_t reg, int32_t data);

    void writeMultiple(uint8_t reg, int8_t *data, uint8_t len);

    uint8_t read8(uint8_t reg);

    void readMultiple(uint8_t reg, uint8_t num, uint8_t *out);

    int32_t readSigned32(uint8_t reg);

    uint32_t readUnsigned32(uint8_t reg);

    int32_t byteArrayToSigned32bitInt(uint8_t *bytes);

    uint32_t byteArrayToUnsigned32bitInt(uint8_t *bytes);

    void clipSigned8(int8_t *val, int8_t min, int8_t max);

private:
    uint8_t i2cAddr;

    static const uint8_t NUM_MFR_BYTES = 8;
    static const uint8_t REGISTER_MFR = 0x08;
    static const uint8_t REGISTER_SENSOR_TYPE = 0x10;
    static const uint8_t NUM_SENSOR_TYPE_BYTES = 8;
};

#endif
