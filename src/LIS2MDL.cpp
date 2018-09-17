/* 
   LIS2MDL.cpp: implementation of LIS2MDL class

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


#include "LIS2MDL.h"
#include <CrossPlatformI2C_Core.h>

LIS2MDL::LIS2MDL(Rate_t odr, float bias[3], float scale[3])
{
    _odr = odr;

    for (uint8_t k=0; k<3; ++k) {
        _bias[k]  = bias[k];
        _scale[k] = scale[k];
    }
}

LIS2MDL::LIS2MDL(Rate_t odr)
{
    float bias[3]  = {0,0,0};
    float scale[3] = {1,1,1};

    LIS2MDL(odr, bias, scale);
}

LIS2MDL::Error_t LIS2MDL::begin(void)
{
    _i2c = cpi2c_open(ADDRESS);

    if (readRegister(WHO_AM_I) != 0x40) {
        return ERROR_ID;
    }

    // Reset device
    uint8_t temp = readRegister(CFG_REG_A);
    writeRegister(CFG_REG_A, temp | 0x20); // Set bit 5 to 1 to reset LIS2MDL
    delay(1);
    writeRegister(CFG_REG_A, temp | 0x40); // Set bit 6 to 1 to boot LIS2MDL
    delay(100); // Wait for all registers to reset 

    // Enable temperature compensation (bit 7 == 1), continuous mode (bits 0:1 == 00)
    writeRegister(CFG_REG_A, 0x80 | _odr<<2);  

    // Enable low pass filter (bit 0 == 1), set to ODR/4
    writeRegister(CFG_REG_B, 0x01);  

    // Enable data ready on interrupt pin (bit 0 == 1), enable block data read (bit 4 == 1)
    writeRegister(CFG_REG_C, 0x01 | 0x10);  

    // Read data to clear interrupt
    int16_t data[4];
    readData(data);

    return selfTest() ? ERROR_NONE : ERROR_SELFTEST;
}


bool LIS2MDL::checkNewData()
{
    return (bool)readRegister(STATUS_REG);   
}

void LIS2MDL::readData(float & mx, float & my, float & mz)
{
    int16_t data[3];
    readData(data);

    mx = (data[0]*RESOLUTION - _bias[0]) * _scale[0];  
    my = (data[1]*RESOLUTION - _bias[1]) * _scale[1];   
    mz = (data[2]*RESOLUTION - _bias[2]) * _scale[2]; 
}

float LIS2MDL::readTemperature(void)
{
    uint8_t rawData[2];  // x/y/z mag register data stored here
    readRegisters((0x80 | TEMP_OUT_L_REG), 2, &rawData[0]);  // Read the 8 raw data registers into data array

    int16_t temp = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value

    return temp / 8.0f + 25.0f; // Convert to Celsius
}

void LIS2MDL::readData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z mag register data stored here
    readRegisters((0x80 | OUTX_L_REG), 8, &rawData[0]);  // Read the 6 raw data registers into data array

    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}



void LIS2MDL::calibrate(void)
{
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};


    for (int ii = 0; ii < 4000; ii++)
    {
        readData(mag_temp);
        for (int jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        delay(12);
    }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    _bias[0] = (float) mag_bias[0] * RESOLUTION;  // save mag biases in G for main program
    _bias[1] = (float) mag_bias[1] * RESOLUTION;   
    _bias[2] = (float) mag_bias[2] * RESOLUTION;  

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0f;

    _scale[0] = avg_rad/((float)mag_scale[0]);
    _scale[1] = avg_rad/((float)mag_scale[1]);
    _scale[2] = avg_rad/((float)mag_scale[2]);
}

bool LIS2MDL::selfTest()
{
    int16_t temp[3] = {0, 0, 0};
    float magTest[3] = {0., 0., 0.};
    float magNom[3] = {0., 0., 0.};
    int32_t sum[3] = {0, 0, 0};

    // first, get average response with self test disabled
    for (int ii = 0; ii < 50; ii++)
    {
        readData(temp);
        sum[0] += temp[0];
        sum[1] += temp[1];
        sum[2] += temp[2];
        delay(50);
    }

    magNom[0] = (float) sum[0] / 50.0f;
    magNom[1] = (float) sum[1] / 50.0f;
    magNom[2] = (float) sum[2] / 50.0f;

    uint8_t c = readRegister(CFG_REG_C);
    writeRegister(CFG_REG_C, c | 0x02); // enable self test
    delay(100); // let mag respond

    sum[0] = 0;
    sum[1] = 0;
    sum[2] = 0;
    for (int ii = 0; ii < 50; ii++)
    {
        readData(temp);
        sum[0] += temp[0];
        sum[1] += temp[1];
        sum[2] += temp[2];
        delay(50);
    }

    magTest[0] = (float) sum[0] / 50.0f;
    magTest[1] = (float) sum[1] / 50.0f;
    magTest[2] = (float) sum[2] / 50.0f;

    writeRegister(CFG_REG_C, c); // return to previous settings/normal mode
    delay(100); // let mag respond

    for (uint8_t k=0; k<3; ++k) {
        float val = (magTest[k] - magNom[k]) * RESOLUTION; 
        if (val < MINVAL || val > MAXVAL) {
            return false;
        }
    }

    return true;
}


// I2C read/write functions for the LIS2MDL
uint8_t LIS2MDL::readRegister(uint8_t subAddress) {
    uint8_t data;
    readRegisters(subAddress, 1, &data);
    return data;
}

void LIS2MDL::writeRegister(uint8_t subAddress, uint8_t data) {
    cpi2c_writeRegister(_i2c, subAddress, data);
}

void LIS2MDL::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) {
    cpi2c_readRegisters(_i2c, subAddress, count, dest);
}
