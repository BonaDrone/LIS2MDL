/* 
   Basic.ino: LIS2MDL basic example

   Copyright (C) 2018 Simon D. Levy

   Additional dependencies:

       https://github.com/simondlevy/LSM6DSM
       https://github.com/simondlevy/LIS2MDL
       https://github.com/simondlevy/CrossPlatformDataBus

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

#include "LIS2MDL.h"

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

// params
static const  LIS2MDL::Rate_t MRATE = LIS2MDL::ODR_50Hz;

// Instantiate LIS2MDL class
static LIS2MDL lis2mdl(MRATE);

static void reportMagnetometer(const char * dim, float val)
{
    Serial.print(dim);
    Serial.print(": ");
    Serial.print(val);
    Serial.print(" milligauss  "); 
}

void setup()
{
    Serial.begin(115200);

#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    delay(100);

    // Start the LIS2MDL
    if (!lis2mdl.begin()) {
        while (true) {
            Serial.println("Unable to connect to LIS2MDL");
        }
    }
}

void loop()
{  
    static float mx, my, mz;

    // Read from LIS2MDL
    if (lis2mdl.checkNewData()) {
        lis2mdl.readData(mx, my, mz);
    }

    // Report at 4 Hz
    static uint32_t msec_prev;
    uint32_t msec_curr = millis();

    if (msec_curr-msec_prev > 250) {

        msec_prev = msec_curr;

        reportMagnetometer("X", mx);
        reportMagnetometer("\tY", my);
        reportMagnetometer("\tZ", mz);

        Serial.println();
    }
}


