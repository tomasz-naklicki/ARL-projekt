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
 */

#pragma once

#include <stdint.h>
#include "bstdr_types.h"
// #include "bmp3_defs.h"

void sensorsBmi088_I2C_deviceInit(struct bmi08_dev *device);
void sensorsBmi088_SPI_deviceInit(struct bmi08_dev *device);

void        bmi088_us_delay(uint32_t period, void *intf_ptr);

// bmi088_com_fptr_t bmi088_burst_read(uint8_t reg_addr, uint8_t *read_data,
//                               uint32_t len, void *intf_ptr);
// bmp3_read_fptr_t bmi088_burst_read(uint8_t dev_id, uint8_t reg_addr,
                            //   uint8_t *reg_data, uint16_t len);
BMI08_INTF_RET_TYPE bmi088_burst_read(uint8_t reg_addr, uint8_t *reg_data,
                                     uint32_t len, void *intf_ptr);

// bmp3_write_fptr_t bmi088_burst_write(uint8_t dev_id, uint8_t reg_addr,
                            //    uint8_t *reg_data, uint16_t len);
BMI08_INTF_RET_TYPE bmi088_burst_write(uint8_t reg_addr, const uint8_t *reg_data,
                                      uint32_t len, void *intf_ptr);
//  BMP3_INTF_RET_TYPE
