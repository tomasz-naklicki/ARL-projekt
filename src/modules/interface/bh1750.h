#pragma once

#include <stdbool.h>
#include "i2cdev.h"

void bh1750TaskInit();
bool bh1750TaskTest();
void bh1750TaskEnqueueData(float value);
bool bh1750_init(I2C_Dev *dev);
bool bh1750_read_lux(I2C_Dev *dev, float *lux);