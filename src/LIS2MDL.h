/* 
   LIS2MDL.h: Header file for LIS2MDL class

   Copyright (C) 2018 Simon D. Levy

   Adapted from https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB

   This file is part of LIS2MDL.

   LIS2MDL is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   LIS2MDL is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with LIS2MDL.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>

// One ifdef needed to support delay() cross-platform
#if defined(ARDUINO)
#include <Arduino.h>

#elif defined(__arm__) 
#if defined(STM32F303)  || defined(STM32F405xx)
extern "C" { void delay(uint32_t msec); }
#else
#include <wiringPi.h>
#endif

#else
void delay(uint32_t msec);
#endif

class LIS2MDL
{
    public:

        typedef enum {

            ODR_10Hz,
            ODR_20Hz,
            ODR_50Hz,
            ODR_100Hz

        } Rate_t;

        typedef enum {

            ERROR_NONE,
            ERROR_CONNECT,
            ERROR_ID,
            ERROR_SELFTEST

        } Error_t;

        static const uint8_t ADDRESS  = 0x1E;

        LIS2MDL(Rate_t odr);

        LIS2MDL(Rate_t odr, float bias[3], float scale[3]);

        Error_t begin(void);

        void calibrate(void);

        bool checkNewData();

        void readData(float & mx, float & my, float & mz);

        float readTemperature();

    private:

        //Register map for LIS2MDL'
        // http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/29/13/d1/e0/9a/4d/4f/30/DM00395193/files/DM00395193.pdf/jcr:content/translations/en.DM00395193.pdf
        static const uint8_t OFFSET_X_REG_L        = 0x45;
        static const uint8_t OFFSET_X_REG_H        = 0x46;
        static const uint8_t OFFSET_Y_REG_L        = 0x47;
        static const uint8_t OFFSET_Y_REG_H        = 0x48;
        static const uint8_t OFFSET_Z_REG_L        = 0x49;
        static const uint8_t OFFSET_Z_REG_H        = 0x4A;
        static const uint8_t WHO_AM_I              = 0x4F;
        static const uint8_t CFG_REG_A             = 0x60;
        static const uint8_t CFG_REG_B             = 0x61;
        static const uint8_t CFG_REG_C             = 0x62;
        static const uint8_t INT_CTRL_REG          = 0x63;
        static const uint8_t INT_SOURCE_REG        = 0x64;
        static const uint8_t INT_THS_L_REG         = 0x65;
        static const uint8_t INT_THS_H_REG         = 0x66;
        static const uint8_t STATUS_REG            = 0x67;
        static const uint8_t OUTX_L_REG            = 0x68;
        static const uint8_t OUTX_H_REG            = 0x69;
        static const uint8_t OUTY_L_REG            = 0x6A;
        static const uint8_t OUTY_H_REG            = 0x6B;
        static const uint8_t OUTZ_L_REG            = 0x6C;
        static const uint8_t OUTZ_H_REG            = 0x6D;
        static const uint8_t TEMP_OUT_L_REG        = 0x6E;
        static const uint8_t TEMP_OUT_H_REG        = 0x6F;

        // Fixed sensitivity and full scale (+/- 49.152 Gauss); 
        static constexpr float RESOLUTION = 0.0015f;  

        // Used in self-test
        static constexpr float MINVAL = .015;
        static constexpr float MAXVAL = .500;

         // Cross-platform support
        uint8_t _i2c;

        // Pass to constructor
        Rate_t _odr;


        // Compute in calibrate() method
        float _bias[3];
        float _scale[3];

        bool selfTest();

        void readData(int16_t * destination);

        void    writeRegister(uint8_t subAddress, uint8_t data);
        uint8_t readRegister(uint8_t subAddress);
        void    readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest);
};
