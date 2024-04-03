/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * sensors_bmi088_i2c.c: I2C backend for the bmi088 sensor
 */

#include "bmi08x.h"
#include "i2cdev.h"
#include "bstdr_types.h"

#include "sensors_bmi088_common.h"

uint8_t acc_dev_add;
uint8_t gyro_dev_add;

void sensorsBmi088_I2C_deviceInit(struct bmi08_dev *device)
{
  // acc_dev_add = (unsigned char) BMI08_ACCEL_I2C_ADDR_PRIMARY;
  // gyro_dev_add = (unsigned char) BMI08_GYRO_I2C_ADDR_SECONDARY;
  // // gyro_dev_add = (unsigned char) BMI08_GYRO_I2C_ADDR_PRIMARY;
  // device->intf_ptr_accel = &acc_dev_add;
  // device->intf_ptr_gyro = &gyro_dev_add;
  // device->intf = BMI08_I2C_INTF;
  // device->read = bmi088_burst_read;
  // device->write = bmi088_burst_write;
  // device->delay_us = bmi088_us_delay;
  // // #define BMI08_READ_WRITE_LEN  UINT8_C(46)
  // // device->read_write_len = BMI08_READ_WRITE_LEN;

  acc_dev_add = BMI08_ACCEL_I2C_ADDR_PRIMARY;
  // gyro_dev_add = BMI08_GYRO_I2C_ADDR_PRIMARY;
  gyro_dev_add = BMI08_GYRO_I2C_ADDR_SECONDARY;
  device->intf = BMI08_I2C_INTF;
  device->read = bmi088_burst_read;
  device->write = bmi088_burst_write;
  /* Selection of bmi085 or bmi088 sensor variant */
  device->variant = (enum bmi08_variant)BMI088_VARIANT;

  /* Assign accel device address to accel interface pointer */
  device->intf_ptr_accel = &acc_dev_add;

  /* Assign gyro device address to gyro interface pointer */
  device->intf_ptr_gyro = &gyro_dev_add;

  /* Configure delay in microseconds */
  device->delay_us = bmi088_us_delay;

  /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
  #define BMI08_READ_WRITE_LEN  UINT8_C(32)
  device->read_write_len = BMI08_READ_WRITE_LEN;

  vTaskDelay(100);
}


