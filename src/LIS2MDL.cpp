/* 
   LIS2MDL.cpp: Implementation of LIS2MDL class

   Copyright (C) 2018 Simon D. Levy

   Adapted from https://github.com/kriswiner/LIS2MDL_LPS22HB

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

LIS2MDL::LIS2MDL(Rate_t rate)
{
    _rate = rate;
}

bool LIS2MDL::begin(uint8_t bus)
{

    _i2c = cpi2c_open(ADDRESS, bus);

    if (_i2c <= 0) {
        return false;
    }

    delay(100);

    if (readRegister(WHO_AM_I) != ID) {
        return false;
    }

    reset();

    delay(100); 

    // enable temperature compensation (bit 7 == 1), continuous mode (bits 0:1 == 00)
    writeRegister(CFG_REG_A, 0x80 | _rate<<2);  

    // enable low pass filter (bit 0 == 1), set to ODR/4
    writeRegister(CFG_REG_B, 0x01);  

    // enable data ready on interrupt pin (bit 0 == 1), enable block data read (bit 4 == 1)
    writeRegister(CFG_REG_C, 0x01 | 0x10);  

    // start with default bias and scale (can be modified by calibration)
    for (uint8_t k=0; k<3; ++k) {
        _bias[k] = 0;
        _scale[k] = 1;
    }

    return true;
}

bool LIS2MDL::checkNewData(void)
{
    return (bool)(readRegister(STATUS_REG)  & 0x08);   
}

void LIS2MDL::reset(void)
{
    // reset device
    uint8_t temp = readRegister(CFG_REG_A);
    writeRegister(CFG_REG_A, temp | 0x20); // Set bit 5 to 1 to reset LIS2MDL
    delay(1);
    writeRegister(CFG_REG_A, temp | 0x40); // Set bit 6 to 1 to boot LIS2MDL
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

    _bias[0] = (float) mag_bias[0] * SCALE;  // save mag biases in G for main program
    _bias[1] = (float) mag_bias[1] * SCALE;   
    _bias[2] = (float) mag_bias[2] * SCALE;  

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

void LIS2MDL::readData(float & x, float & y, float & z)
{
    int16_t data[3];

    readData(data);     

    x = 1000 * (data[0]*SCALE - _bias[0]) * _scale[0];  
    y = 1000 * (data[1]*SCALE - _bias[1]) * _scale[1];   
    z = 1000 * (data[2]*SCALE - _bias[2]) * _scale[2];  
}

float LIS2MDL::readTemperature(void)
{
    uint8_t data[2];  

    readRegisters((0x80 | TEMP_OUT_L_REG), 2, data);  

    return (((int16_t)data[1] << 8) | data[0]) / 8.0f + 25.0f; ;
}

void LIS2MDL::readData(int16_t data[3])
{
  uint8_t rawData[6];  

  readRegisters((0x80 | OUTX_L_REG), 8, rawData);  

  data[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       
  data[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  data[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

uint8_t LIS2MDL::readRegister(uint8_t subAddress)
{
    uint8_t data=0;
    readRegisters(subAddress, 1, &data);
    return data;
}

void LIS2MDL::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    cpi2c_readRegisters(_i2c, subAddress, count, dest);
}

void LIS2MDL::writeRegister(uint8_t subAddress, uint8_t data)
{
    cpi2c_writeRegister(_i2c, subAddress, data);
}


