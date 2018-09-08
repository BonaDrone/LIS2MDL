/* 
   Basic.cpp: LIS2MDL basic example

   Copyright (C) 2018 Simon D. Levy

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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

// in timing.cpp
uint32_t millis(void);

// params
static const  LIS2MDL::Rate_t MRATE = LIS2MDL::ODR_50Hz;

// Instantiate LIS2MDL class
static LIS2MDL lis2mdl(MRATE);

static void report(const char * dim, float val)
{
    printf("%s: %f milligauss ", dim, val);
}

void setup()
{
    if (!lis2mdl.begin(0)) {
        while (true) {
            printf("Unable to connect to LIS2MDL\n");
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

        report("X", mx);
        report("\tY", my);
        report("\tZ", mz);

        printf("\n");
    }
}


