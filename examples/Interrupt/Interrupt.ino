/* 
   Interrupt.ino: LIS2MDL interrupt-driven example

   Copyright (C) 2018 Simon D. Levy

   This file is part of LSM6DSM.

   LSM6DSM is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   LSM6DSM is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with LSM6DSM.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "LIS2MDL.h"

#include <Wire.h>

static uint8_t LED_PIN = 38;

static uint8_t INTERRUPT_PIN  = 43;

// These were computed previously by Kris.  
// We use them here to avoid having to calibrate.
static float BIAS[3]  = {0.814, -0.093, -0.579};
static float SCALE[3] = {1.11,   1.02,   0.90}; 

// Specify sensor parameters (sample rate is twice the bandwidth)
// choices are: ODR_10Hz, MOIDR_20Hz, ODR_50 Hz and ODR_100Hz
static LIS2MDL::Rate_t ODR = LIS2MDL::ODR_100Hz;

static LIS2MDL lis2mdl = LIS2MDL(ODR, BIAS, SCALE); 

static bool newData = false;

static  void interruptHandler(void)
{
    newData = true;
}

static void error(const char * msg)
{
    while (true) {
        Serial.print("Error: ");
        Serial.println(msg);
    }
}

void setup() 
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // start with led off

    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN , interruptHandler, RISING);  

    Wire.begin(TWI_PINS_20_21); // set master mode 
    Wire.setClock(400000); // I2C frequency at 400 kHz  
    delay(100);

    switch (lis2mdl.begin()) {

        case LIS2MDL::ERROR_CONNECT:
            error("no connection");
            break;

        case LIS2MDL::ERROR_ID:
            error("bad ID");
            break;

        case LIS2MDL::ERROR_SELFTEST:
            error("failed self-test");
            break;

         case LIS2MDL::ERROR_NONE:
            break;

    }

    digitalWrite(LED_PIN, LOW);

    // Un-comment the following lines to calibrate
    /*
    Serial.println("Calibrate: move all around to sample the complete response surface!");
    delay(2000);
    float bias[3] = {0,0,0}, scale[3] = {0,0,0};
    lis2mdl.calibrate(bias, scale);
    Serial.println("Mag Calibration done!");
    Serial.println("mag biases (mG)");
    Serial.println(1000.0f * bias[0]);
    Serial.println(1000.0f * bias[1]);
    Serial.println(1000.0f * bias[2]); 
    Serial.println("mag scale (mG)");
    Serial.println(scale[0]);
    Serial.println(scale[1]);
    Serial.println(scale[2]); 
    delay(2000); 
    */

    // Read once to ensure interrupts will work
    float mx=0, my=0, mz=0;
    lis2mdl.readData(mx, my, mz);

    // Turn LED on
    digitalWrite(LED_PIN, LOW);
}


void loop() 
{
    // On interrupt, read data
    if (newData) {   

        newData = false;     // reset newData flag

        float mx=0, my=0, mz=0;
        lis2mdl.readData(mx, my, mz);

        Serial.print("mx = ");
        Serial.print((int)1000*mx);  
        Serial.print(" my = ");
        Serial.print((int)1000*my); 
        Serial.print(" mz = ");
        Serial.print((int)1000*mz);
        Serial.println(" mG");

        Serial.print("Mag temperature is ");  
        Serial.print(lis2mdl.readTemperature(), 1);  
        Serial.println(" degrees C\n");
    }
}
