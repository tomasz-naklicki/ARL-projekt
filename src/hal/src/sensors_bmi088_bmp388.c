/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2018 Bitcraze AB
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
 * sensors_bmi088_bmp388.c: IMU sensor driver for the *88 bosch sensors
 */

#define DEBUG_MODULE "IMU"

#include <math.h>

#include "sensors_bmi088_bmp388.h"
#include "stm32fxxx.h"

#include "imu.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "i2cdev.h"
#include "bmi08x.h"
#include "bmp3.h"
#include "bstdr_types.h"
#include "static_mem.h"
#include "estimator.h"

#include "sensors_bmi088_common.h"
#include "platform_defaults.h"

#define GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES

#define SENSORS_READ_RATE_HZ            1000
#define SENSORS_STARTUP_TIME_MS         1000
#define SENSORS_READ_BARO_HZ            50
#define SENSORS_READ_MAG_HZ             20
#define SENSORS_DELAY_BARO              (SENSORS_READ_RATE_HZ/SENSORS_READ_BARO_HZ)
#define SENSORS_DELAY_MAG               (SENSORS_READ_RATE_HZ/SENSORS_READ_MAG_HZ)

#define SENSORS_BMI088_GYRO_FS_CFG      BMI08_GYRO_RANGE_2000_DPS
#define SENSORS_BMI088_DEG_PER_LSB_CFG  (2.0f *2000.0f) / 65536.0f

#define SENSORS_BMI088_ACCEL_CFG        24
#define SENSORS_BMI088_ACCEL_FS_CFG     BMI088_ACCEL_RANGE_24G
#define SENSORS_BMI088_G_PER_LSB_CFG    (2.0f * (float)SENSORS_BMI088_ACCEL_CFG) / 65536.0f
#define SENSORS_BMI088_1G_IN_LSB        (65536 / SENSORS_BMI088_ACCEL_CFG / 2)

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT   M2T(1000) // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX          5.0f      // Max degrees off

#define GYRO_NBR_OF_AXES                3
#define GYRO_MIN_BIAS_TIMEOUT_MS        M2T(1*1000)

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES  512

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE              100
#define GYRO_VARIANCE_THRESHOLD_X       (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y       (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z       (GYRO_VARIANCE_BASE)

#define SENSORS_ACC_SCALE_SAMPLES  200


typedef struct
{
  Axis3f     bias;
  Axis3f     variance;
  Axis3f     mean;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

/* initialize necessary variables */
static struct bmi08_dev bmi088Dev;
static struct bmp3_dev   bmp388Dev;

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle sensorsDataReady;
static StaticSemaphore_t sensorsDataReadyBuffer;
static xSemaphoreHandle dataReady;
static StaticSemaphore_t dataReadyBuffer;

static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
NO_DMA_CCM_SAFE_ZERO_INIT static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined (GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f gyroBiasStdDev;
#endif
static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;
static bool accScaleFound = false;
static uint32_t accScaleSumCount = 0;

// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);

static bool isBarometerPresent = false;
static uint8_t baroMeasDelayMin = SENSORS_DELAY_BARO;

// IMU alignment Euler angles
static float imuPhi = IMU_PHI;
static float imuTheta = IMU_THETA;
static float imuPsi = IMU_PSI;

static float R[3][3];

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

#ifdef GYRO_GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz,  Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static void sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAlignToAirframe(Axis3f* in, Axis3f* out);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

// Communication routines

/*!
 * @brief Generic burst read
 *
 * @param [out] dev_id I2C address, SPI chip select or user desired identifier
 *
 * @return Zero if successful, otherwise an error code
 */
BMI08_INTF_RET_TYPE bmi088_burst_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  /**< Burst read code comes here */
  uint8_t device_addr = *(uint8_t*)intf_ptr;
  if (i2cdevReadReg8(I2C3_DEV, device_addr, reg_addr, (uint16_t) len, reg_data))
  {
    return BMI08_OK;
  }
  else
  {
    return BSTDR_E_CON_ERROR;
  }
}

/*!
 * @brief Generic burst write
 *
 * @param [out] dev_id I2C address, SPI chip select or user desired identifier
 *
 * @return Zero if successful, otherwise an error code
 */
BMI08_INTF_RET_TYPE bmi088_burst_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  /**< Burst write code comes here */
  uint8_t device_addr = *(uint8_t*)intf_ptr;
  if (i2cdevWriteReg8(I2C3_DEV, device_addr, reg_addr, (uint16_t) len, reg_data))
  {
    return BSTDR_OK;
  }
  else
  {
    return BSTDR_E_CON_ERROR;
  }
}

/*!
 * @brief Generic burst read
 *
 * @param [in] period Delay period in microseconds
 *
 * @return None
 */
void bmi088_us_delay(uint32_t period, void *intf_ptr)
{
  (void)intf_ptr;
  /**< Delay code comes */
  uint32_t milisecond_period = period / 1000;
  // but minimal delay is 1ms
  if (milisecond_period < 1)
  {
    milisecond_period = 1;
  }
  vTaskDelay(M2T(milisecond_period)); // Delay a while to let the device stabilize.
  // todo; check this period
}

static uint16_t sensorsGyroGet(Axis3i16* dataOut)
{
  int8_t result;
  result = bmi08g_get_data((struct bmi08_sensor_data*)dataOut, &bmi088Dev);
  if (result == BMI08_OK) {
    DEBUG_PRINT("Got gyro data [OK]\n");
  }
  else {
    DEBUG_PRINT("Got gyro data [FAIL]\n");
  }
  return result;
}

static void sensorsAccelGet(Axis3i16* dataOut)
{
  int8_t result;
  result = bmi08a_get_data((struct bmi08_sensor_data*)dataOut, &bmi088Dev);
  if (result == BMI08_OK) {
    DEBUG_PRINT("Got accel data [OK]\n");
  }
  else {
    DEBUG_PRINT("Got accel data [FAIL]\n");
  }
}

static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature)
{
  baroScaled->pressure = pressure*0.01f;
  baroScaled->temperature = temperature;
  baroScaled->asl = ((powf((1015.7f / baroScaled->pressure), 0.1902630958f)
      - 1.0f) * (25.0f + 273.15f)) / 0.0065f;
}

bool sensorsBmi088Bmp388ReadGyro(Axis3f *gyro)
{

  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsBmi088Bmp388ReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsBmi088Bmp388ReadMag(Axis3f *mag)
{
  return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsBmi088Bmp388ReadBaro(baro_t *baro)
{
  return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsBmi088Bmp388Acquire(sensorData_t *sensors)
{
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsBmi088Bmp388AreCalibrated()
{
  return gyroBiasFound;
}

static void sensorsTask(void *param)
{
  systemWaitStart();

  Axis3f gyroScaledIMU;
  Axis3f accScaledIMU;
  Axis3f accScaled;
  measurement_t measurement;
  /* wait an additional second the keep bus free
   * this is only required by the z-ranger, since the
   * configuration will be done after system start-up */
  //vTaskDelayUntil(&lastWakeTime, M2T(1500));
  while (1)
  {
    DEBUG_PRINT("sensorTaskPulse\n");
    
    if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
    {
    // vTaskDelay(M2T(1));
      // ASSERT(0);
      sensorData.interruptTimestamp = imuIntTimestamp;

      /* get data from chosen sensors */
      sensorsGyroGet(&gyroRaw);
      sensorsAccelGet(&accelRaw);
      DEBUG_PRINT("BMI088 Accel: x=%d y=%d z=%d\n", accelRaw.x, accelRaw.y, accelRaw.z);
      DEBUG_PRINT("BMI088 Gyro: x=%d y=%d z=%d\n", gyroRaw.x, gyroRaw.y, gyroRaw.z);

      /* calibrate if necessary */
#ifdef GYRO_BIAS_LIGHT_WEIGHT
      gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
      gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif
      if (gyroBiasFound)
      {
         processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
      }

      /* Gyro */
      gyroScaledIMU.x =  (gyroRaw.x - gyroBias.x) * SENSORS_BMI088_DEG_PER_LSB_CFG;
      gyroScaledIMU.y =  (gyroRaw.y - gyroBias.y) * SENSORS_BMI088_DEG_PER_LSB_CFG;
      gyroScaledIMU.z =  (gyroRaw.z - gyroBias.z) * SENSORS_BMI088_DEG_PER_LSB_CFG;
      sensorsAlignToAirframe(&gyroScaledIMU, &sensorData.gyro);
      applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensorData.gyro);

      measurement.type = MeasurementTypeGyroscope;
      measurement.data.gyroscope.gyro = sensorData.gyro;
      estimatorEnqueue(&measurement);

      /* Acelerometer */
      accScaledIMU.x = accelRaw.x * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
      accScaledIMU.y = accelRaw.y * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
      accScaledIMU.z = accelRaw.z * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
      sensorsAlignToAirframe(&accScaledIMU, &accScaled);
      sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
      applyAxis3fLpf((lpf2pData*)(&accLpf), &sensorData.acc);

      measurement.type = MeasurementTypeAcceleration;
      measurement.data.acceleration.acc = sensorData.acc;
      estimatorEnqueue(&measurement);
    }
    // DEBUG_PRINT("sensorTaskPulseEnd\n");
    if (isBarometerPresent)
    {
      static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
      if (--baroMeasDelay == 0)
      {
        uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;
        struct bmp3_data data;
        baro_t* baro388 = &sensorData.baro;
        /* Temperature and Pressure data are read and stored in the bmp3_data instance */
        bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);
        sensorsScaleBaro(baro388, data.pressure, data.temperature);

        measurement.type = MeasurementTypeBarometer;
        measurement.data.barometer.baro = sensorData.baro;
        estimatorEnqueue(&measurement);

        baroMeasDelay = baroMeasDelayMin;
      }
    }
    // DEBUG_PRINT("pastBarometer\n");
    xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
    xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
    if (isBarometerPresent)
    {
      xQueueOverwrite(barometerDataQueue, &sensorData.baro);
    }

    xSemaphoreGive(dataReady);
  }
}

void sensorsBmi088Bmp388WaitDataReady(void)
{
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

// static double lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
// {
//     double power = 2;

//     double half_scale = ((pow((double)power, (double)bit_width) / 2.0));

//     return (9.81 * val * g_range) / half_scale;
// }

// static double lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
// {
//     double power = 2;

//     double half_scale = ((pow((double)power, (double)bit_width) / 2.0));

//     return ((double)dps / (half_scale)) * (val);
// }


static void sensorsDeviceInit(void)
{
  if (isInit)
    return;

  bstdr_ret_t rslt;
  isBarometerPresent = false;

  // Wait for sensors to startup
  vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS));

  /* BMI088
   * The bmi088Dev structure should have been filled in by the backend
   * (i2c/spi) drivers at this point.
  */
  // bmi088Dev.variant = (enum bmi08_variant)BMI088_VARIANT;
  rslt = bmi08xa_init(&bmi088Dev);
  // rslt = bmi08a_init(&bmi088Dev);
  rslt = BMI08_OK;

  if (rslt == BMI08_OK)
  {
      rslt |= bmi08g_init(&bmi088Dev);
  }

  if (rslt == BMI08_OK)
  {
      rslt |= bmi08a_load_config_file(&bmi088Dev);
  }

  if (rslt == BMI08_OK)
  {
    bmi088Dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;

    if (bmi088Dev.variant == BMI085_VARIANT)
    {
        bmi088Dev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
    }
    else if (bmi088Dev.variant == BMI088_VARIANT)
    {
        bmi088Dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
    }
    DEBUG_PRINT("Variant: %d\n", bmi088Dev.variant);

    bmi088Dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
    // bmi088Dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */
    bmi088Dev.accel_cfg.bw = BMI08_ACCEL_BW_OSR4;

    rslt |= bmi08a_set_power_mode(&bmi088Dev);
    // //bmi08_error_codes_print_result("bmi08a_set_power_mode", rslt);

    rslt |= bmi08xa_set_meas_conf(&bmi088Dev);
    // //bmi08_error_codes_print_result("bmi08xa_set_meas_conf", rslt);

    bmi088Dev.gyro_cfg.odr = BMI08_GYRO_BW_116_ODR_1000_HZ;
    bmi088Dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    bmi088Dev.gyro_cfg.bw = BMI08_GYRO_BW_116_ODR_1000_HZ;
    bmi088Dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

    rslt |= bmi08g_set_power_mode(&bmi088Dev);
    // //bmi08_error_codes_print_result("bmi08g_set_power_mode", rslt);

    rslt |= bmi08g_set_meas_conf(&bmi088Dev);
    // //bmi08_error_codes_print_result("bmi08g_set_meas_conf", rslt);
  }

  if (rslt == BMI08_OK)
  {
    DEBUG_PRINT("BMI088 connection [OK]\n");
    isInit = true;
  }
  else
  {
    DEBUG_PRINT("BMI088 connection [FAIL]\n");
    isInit = false;
    return;
  }

  // interrupt
  uint8_t data = 0;

  // /*! bmi08 accel int config */
  // struct bmi08_accel_int_channel_cfg accel_int_config;

  /*! bmi08 gyro int config */
  struct bmi08_gyro_int_channel_cfg gyro_int_config;

  // /* Set accel interrupt pin configuration */
  // accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
  // accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
  // accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
  // accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
  // accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

  // /* Enable accel data ready interrupt channel */
  // rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg*)&accel_int_config, &bmi088Dev);

  if (rslt == BMI08_OK)
  {
      /* Set gyro interrupt pin configuration */
      gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
      gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
      gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
      gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
      gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

      /* Enable gyro data ready interrupt channel */
      rslt |= bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config, &bmi088Dev);

      rslt |= bmi08g_get_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data, 1, &bmi088Dev);
  }

  if (rslt == BMI08_OK)
  {
    DEBUG_PRINT("BMI088 interrupt configuration [OK]\n");
  }
  else
  {
    DEBUG_PRINT("BMI088 interrupt configuration [FAIL]\n");
    isInit = false;
    return;
  }

  // // do some tests
  // uint8_t times_to_read = 0;
  // double x = 0.0, y = 0.0, z = 0.0;
  // uint8_t status = 0;
  // /*! @brief variable to hold the bmi08 accel data */
  // struct bmi08_sensor_data bmi08_accel;

  // /*! @brief variable to hold the bmi08 gyro data */
  // struct bmi08_sensor_data bmi08_gyro;

  // if (rslt == BMI08_OK)
  // {
  //     if (bmi088Dev.accel_cfg.power == BMI08_ACCEL_PM_ACTIVE)
  //     {
  //         DEBUG_PRINT("\nACCEL DATA\n");
  //         DEBUG_PRINT("Accel data in LSB units and Gravity data in m/s^2\n");
  //         DEBUG_PRINT("Accel data range : 16G for BMI085 and 24G for BMI088\n\n");

  //         DEBUG_PRINT("Sample_Count, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z\n");

  //         while (times_to_read < 10)
  //         {
  //             rslt = bmi08a_get_data_int_status(&status, &bmi088Dev);
  //             //bmi08_error_codes_print_result("bmi08a_get_data_int_status", rslt);

  //             if (status & BMI08_ACCEL_DATA_READY_INT)
  //             {
  //                 rslt = bmi08a_get_data(&bmi08_accel, &bmi088Dev);
  //                 //bmi08_error_codes_print_result("bmi08a_get_data", rslt);

  //                 if (bmi088Dev.variant == BMI085_VARIANT)
  //                 {
  //                     /* Converting lsb to meter per second squared for 16 bit accelerometer at 16G range. */
  //                     x = lsb_to_mps2(bmi08_accel.x, 16, 16);
  //                     y = lsb_to_mps2(bmi08_accel.y, 16, 16);
  //                     z = lsb_to_mps2(bmi08_accel.z, 16, 16);
  //                 }
  //                 else if (bmi088Dev.variant == BMI088_VARIANT)
  //                 {
  //                     /* Converting lsb to meter per second squared for 16 bit accelerometer at 24G range. */
  //                     x = lsb_to_mps2(bmi08_accel.x, 24, 16);
  //                     y = lsb_to_mps2(bmi08_accel.y, 24, 16);
  //                     z = lsb_to_mps2(bmi08_accel.z, 24, 16);
  //                 }

  //                 DEBUG_PRINT("%d, %5d, %5d, %5d, %4.2f, %4.2f, %4.2f\n",
  //                         times_to_read,
  //                         bmi08_accel.x,
  //                         bmi08_accel.y,
  //                         bmi08_accel.z,
  //                         x,
  //                         y,
  //                         z);

  //                 times_to_read = times_to_read + 1;
  //                 vTaskDelay(M2T(10));
  //             }
  //         }
  //     }

  //     if (bmi088Dev.gyro_cfg.power == BMI08_GYRO_PM_NORMAL)
  //     {
  //         times_to_read = 0;

  //         DEBUG_PRINT("\n\nGYRO DATA\n");
  //         DEBUG_PRINT("Gyro data in LSB units and degrees per second\n");
  //         DEBUG_PRINT("Gyro data range : 250 dps for BMI085 and BMI088\n\n");

  //         DEBUG_PRINT("Sample_Count, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyr_DPS_X, Gyr_DPS_Y, Gyr_DPS_Z\n");

  //         while (times_to_read < 10)
  //         {
  //             rslt = bmi08g_get_data_int_status(&status, &bmi088Dev);
  //             //bmi08_error_codes_print_result("bmi08g_get_data_int_status", rslt);

  //             if (status & BMI08_GYRO_DATA_READY_INT)
  //             {
  //                 rslt = bmi08g_get_data(&bmi08_gyro, &bmi088Dev);
  //                 //bmi08_error_codes_print_result("bmi08g_get_data", rslt);

  //                 /* Converting lsb to degree per second for 16 bit gyro at 250 dps range. */
  //                 x = lsb_to_dps(bmi08_gyro.x, (float)250, 16);
  //                 y = lsb_to_dps(bmi08_gyro.y, (float)250, 16);
  //                 z = lsb_to_dps(bmi08_gyro.z, (float)250, 16);

  //                 DEBUG_PRINT("%d, %5d, %5d, %5d, %4.2f, %4.2f, %4.2f\n",
  //                         times_to_read,
  //                         bmi08_gyro.x,
  //                         bmi08_gyro.y,
  //                         bmi08_gyro.z,
  //                         x,
  //                         y,
  //                         z);

  //                 times_to_read = times_to_read + 1;
  //                 vTaskDelay(M2T(10));
  //             }
  //         }
  //     }
  // }

//   /* BMI088 GYRO */
//   rslt = bmi08g_init(&bmi088Dev); // initialize the device
//   if (rslt == BSTDR_OK)
//   {
//     struct bmi08_gyro_int_channel_cfg intConfig;

//     DEBUG_PRINT("BMI088 Gyro connection [OK].\n");
//     /* set power mode of gyro */
//     bmi088Dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
//     /* set bandwidth and range of gyro */
//     bmi088Dev.gyro_cfg.bw = BMI08_GYRO_BW_116_ODR_1000_HZ;
//     bmi088Dev.gyro_cfg.range = SENSORS_BMI088_GYRO_FS_CFG;
//     bmi088Dev.gyro_cfg.odr = BMI08_GYRO_BW_116_ODR_1000_HZ;
//     rslt |= bmi08g_set_power_mode(&bmi088Dev);
//     rslt |= bmi08g_set_meas_conf(&bmi088Dev);

//     intConfig.int_channel = BMI08_INT_CHANNEL_3;
//     intConfig.int_type = BMI08_GYRO_INT_DATA_RDY;
//     intConfig.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
//     intConfig.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
//     intConfig.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
//     /* Setting the interrupt configuration */
//     rslt = bmi08g_set_int_config(&intConfig, &bmi088Dev);
//     if (rslt != BSTDR_OK)
//     {
//       DEBUG_PRINT("BMI088 Gyro interrupt configuration [FAIL]\n");
//     }
//     else {
//       DEBUG_PRINT("BMI088 Gyro interrupt configuration [OK]\n");
    
//     }

//     bmi088Dev.delay_us(50000, bmi088Dev.intf_ptr_gyro);
//     struct bmi08_sensor_data gyr;
//     rslt |= bmi08g_get_data(&gyr, &bmi088Dev);
//   }
//   else
//   {
// #ifndef SENSORS_IGNORE_IMU_FAIL
//     DEBUG_PRINT("RESULT: %d\n", rslt);
//     DEBUG_PRINT("BMI088 Gyro connection [FAIL]\n");
//     isInit = false;
// #endif
//   }

//   /* BMI088 ACCEL */
//   // rslt |= bmi08a_switch_control(&bmi088Dev, BMI08_ACCEL_POWER_ENABLE); // this is set below in cfg power
//   bmi088Dev.delay_us(5000, bmi088Dev.intf_ptr_accel);

//   // bmi088Dev.variant = (enum bmi08_variant)BMI088_VARIANT;
//   // rslt = bmi08xa_init(&bmi088Dev); // initialize the device
//   rslt |= bmi08a_init(&bmi088Dev); // initialize the device
//   rslt |= bmi08a_load_config_file(&bmi088Dev);
//   if (rslt == BSTDR_OK)
//   {
//     DEBUG_PRINT("BMI088 Accel connection [OK]\n");
//     /* set power mode of accel */
//     bmi088Dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
//     bmi088Dev.delay_us(10000, bmi088Dev.intf_ptr_accel);

//     /* set bandwidth and range of accel */
//     bmi088Dev.accel_cfg.bw = BMI08_ACCEL_BW_OSR4;
//     bmi088Dev.accel_cfg.range = SENSORS_BMI088_ACCEL_FS_CFG;
//     bmi088Dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;
//     rslt |= bmi08a_set_power_mode(&bmi088Dev);
//     rslt |= bmi08a_set_meas_conf(&bmi088Dev);
//     rslt |= bmi08xa_set_meas_conf(&bmi088Dev);

//     struct bmi08_sensor_data acc;
//     rslt |= bmi08a_get_data(&acc, &bmi088Dev);
//     DEBUG_PRINT("BMI088 Accel: x=%d y=%d z=%d\n", acc.x, acc.y, acc.z);
//   }
//   else
//   {
// #ifndef SENSORS_IGNORE_IMU_FAIL
//     DEBUG_PRINT("BMI088 Accel connection [FAIL]\n");
//     isInit = false;
// #endif
//   }

  /* BMP388 */
  uint8_t intf;
  intf = (unsigned char) BMP3_ADDR_I2C_SEC;
  bmp388Dev.intf_ptr = &intf;
  bmp388Dev.intf = BMP3_I2C_INTF;
  bmp388Dev.read = bmi088_burst_read;
  bmp388Dev.write = bmi088_burst_write;
  bmp388Dev.delay_us = bmi088_us_delay; // todo; verify this is correct

  int i = 3;
  do {
    bmp388Dev.delay_us(1000, bmp388Dev.intf_ptr);
    // For some reason it often doesn't work first time
    rslt = bmp3_init(&bmp388Dev);
  } while (rslt != BMP3_OK && i-- > 0);

  if (rslt == BMP3_OK)
  {
    isBarometerPresent = true;
    DEBUG_PRINT("BMP388 I2C connection [OK]\n");
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;
    /* Select the pressure and temperature sensor to be enabled */
    struct bmp3_settings settings = { 0 };
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr = BMP3_ODR_50_HZ;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER;
    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &bmp388Dev);

    /* Set the power mode to normal mode */
    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &bmp388Dev);


    bmp388Dev.delay_us(20000, bmp388Dev.intf_ptr); // wait before first read out
    // read out data
    /* Variable used to select the sensor component */
    uint8_t sensor_comp;
    /* Variable used to store the compensated data */
    struct bmp3_data data;

    /* Sensor component selection */
    sensor_comp = BMP3_PRESS | BMP3_TEMP;
    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
    rslt = bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);

    /* Print the temperature and pressure data */
    DEBUG_PRINT("BMP388 T:%0.2f  P:%0.2f\n",data.temperature, data.pressure/100.0);
    baroMeasDelayMin = SENSORS_DELAY_BARO;
  }
  else
  {
#ifndef CONFIG_SENSORS_IGNORE_BAROMETER_FAIL
    DEBUG_PRINT("BMP388 I2C connection [FAIL]\n");
    isInit = false;
    return;
#endif
  }

  // Init second order filer for accelerometer and gyro
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
  }

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI / 180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI / 180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI / 180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI / 180);

  isInit = true;
}

static void sensorsTaskInit(void)
{
  accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
  gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
  magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
  barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}

static void sensorsInterruptInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  sensorsDataReady = xSemaphoreCreateBinaryStatic(&sensorsDataReadyBuffer);
  dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

  // Enable the interrupt on PC14
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);

  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  portDISABLE_INTERRUPTS();
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_Line14);
  portENABLE_INTERRUPTS();
}

static void sensorsBmi088Bmp388Init(void)
{
  sensorsBiasObjInit(&gyroBiasRunning);
  sensorsDeviceInit();
  sensorsInterruptInit();
  sensorsTaskInit();
}

void sensorsBmi088Bmp388Init_SPI(void)
{
  if (isInit)
    {
      return;
    }

    DEBUG_PRINT("BMI088: Using SPI interface.\n");
    sensorsBmi088_SPI_deviceInit(&bmi088Dev);
    sensorsBmi088Bmp388Init();
}

void sensorsBmi088Bmp388Init_I2C(void)
{
  if (isInit)
  {
    return;
  }

  DEBUG_PRINT("BMI088: Using I2C interface.\n");
  sensorsBmi088_I2C_deviceInit(&bmi088Dev);
  sensorsBmi088Bmp388Init();
}

static bool gyroSelftest()
{
  bool testStatus = true;

  int i = 3;
  uint16_t readResult = BMI08_OK;
  do {
    // For some reason it often doesn't work first time on the Roadrunner
    readResult = sensorsGyroGet(&gyroRaw);
  } while (readResult != BMI08_OK && i-- > 0);

  if ((readResult != BMI08_OK) || (gyroRaw.x == 0 && gyroRaw.y == 0 && gyroRaw.z == 0))
  {
    DEBUG_PRINT("BMI088 gyro returning x=0 y=0 z=0 [FAILED]\n");
    testStatus = false;
  }
  
  int8_t gyroResult = 0;
  gyroResult = bmi08g_perform_selftest(&bmi088Dev);
  if (gyroResult == BMI08_OK)
  {
    DEBUG_PRINT("BMI088 gyro self-test [OK]\n");
  }
  else
  {
    DEBUG_PRINT("BMI088 gyro self-test [FAILED]\n");
    testStatus = false;
  }

  return testStatus;
}

bool sensorsBmi088Bmp388Test(void)
{
  bool testStatus = true;

  if (!isInit)
  {
    DEBUG_PRINT("Uninitialized\n");
    testStatus = false;
  }

  if (! gyroSelftest())
  {
    testStatus = false;
  }

  return testStatus;
}

/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
  if (!accScaleFound)
  {
    accScaleSum += sqrtf(powf(ax * SENSORS_BMI088_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_BMI088_G_PER_LSB_CFG, 2) + powf(az * SENSORS_BMI088_G_PER_LSB_CFG, 2));  // todo; review if this needs an update
    accScaleSumCount++;

    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
    {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accScaleFound = true;
    }
  }

  return accScaleFound;
}

#ifdef GYRO_BIAS_LIGHT_WEIGHT

#define SENSORS_BIAS_SAMPLES       1000
/**
 * Calculates the bias out of the first SENSORS_BIAS_SAMPLES gathered. Requires no buffer
 * but needs platform to be stable during startup.
 */
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  static uint32_t gyroBiasSampleCount = 0;
  static bool gyroBiasNoBuffFound = false;
  static Axis3i64 gyroBiasSampleSum;
  static Axis3i64 gyroBiasSampleSumSquares;

  if (!gyroBiasNoBuffFound)
  {
    // If the gyro has not yet been calibrated:
    // Add the current sample to the running mean and variance
    gyroBiasSampleSum.x += gx;
    gyroBiasSampleSum.y += gy;
    gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
    gyroBiasSampleSumSquares.x += gx * gx;
    gyroBiasSampleSumSquares.y += gy * gy;
    gyroBiasSampleSumSquares.z += gz * gz;
#endif
    gyroBiasSampleCount += 1;

    // If we then have enough samples, calculate the mean and standard deviation
    if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES)
    {
      gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;

#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
      gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->x * gyroBiasOut->x));
      gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->y * gyroBiasOut->y));
      gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->z * gyroBiasOut->z));
#endif
      gyroBiasNoBuffFound = true;
    }
  }

  return gyroBiasNoBuffFound;
}
#else
/**
 * Calculates the bias first when the gyro variance is below threshold. Requires a buffer
 * but calibrates platform first when it is stable.
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

  if (!gyroBiasRunning.isBiasValueFound)
  {
    sensorsFindBiasValue(&gyroBiasRunning);
    if (gyroBiasRunning.isBiasValueFound)
    {
      soundSetEffect(SND_CALIB);
      ledseqRun(&seq_calibrated);
    }
  }

  gyroBiasOut->x = gyroBiasRunning.bias.x;
  gyroBiasOut->y = gyroBiasRunning.bias.y;
  gyroBiasOut->z = gyroBiasRunning.bias.z;
  DEBUG_PRINT("gyroBias found: %d\n", gyroBiasRunning.isBiasValueFound);
  return gyroBiasRunning.isBiasValueFound;
}
#endif

static void sensorsBiasObjInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
  uint32_t i;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  meanOut->x = (float) sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float) sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float) sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;

  varOut->x = sumSq[0] / SENSORS_NBR_OF_BIAS_SAMPLES - meanOut->x * meanOut->x;
  varOut->y = sumSq[1] / SENSORS_NBR_OF_BIAS_SAMPLES - meanOut->y * meanOut->y;
  varOut->z = sumSq[2] / SENSORS_NBR_OF_BIAS_SAMPLES - meanOut->z * meanOut->z;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool sensorsFindBiasValue(BiasObj* bias)
{
  static int32_t varianceSampleTime;
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

    if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = bias->mean.x;
      bias->bias.y = bias->mean.y;
      bias->bias.z = bias->mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}

bool sensorsBmi088Bmp388ManufacturingTest(void)
{
  bool testStatus = true;
  if (! gyroSelftest())
  {
    testStatus = false;
  }

  int8_t accResult = 0;
  accResult = bmi08xa_perform_selftest(&bmi088Dev);
  if (accResult == BMI08_OK)
  {
    DEBUG_PRINT("BMI088 acc self-test [OK]\n");
  }
  else
  {
    DEBUG_PRINT("BMI088 acc self-test [FAILED]\n");
    testStatus = false;
  }

  return testStatus;
}

/**
 * Align the sensors to the Airframe axes
 */
static void sensorsAlignToAirframe(Axis3f* in, Axis3f* out)
{
  // IMU alignment
  static float sphi, cphi, stheta, ctheta, spsi, cpsi;

  sphi   = sinf(imuPhi * (float) M_PI / 180);
  cphi   = cosf(imuPhi * (float) M_PI / 180);
  stheta = sinf(imuTheta * (float) M_PI / 180);
  ctheta = cosf(imuTheta * (float) M_PI / 180);
  spsi   = sinf(imuPsi * (float) M_PI / 180);
  cpsi   = cosf(imuPsi * (float) M_PI / 180);

  R[0][0] = ctheta * cpsi;
  R[0][1] = ctheta * spsi;
  R[0][2] = -stheta;
  R[1][0] = sphi * stheta * cpsi - cphi * spsi;
  R[1][1] = sphi * stheta * spsi + cphi * cpsi;
  R[1][2] = sphi * ctheta;
  R[2][0] = cphi * stheta * cpsi + sphi * spsi;
  R[2][1] = cphi * stheta * spsi - sphi * cpsi;
  R[2][2] = cphi * ctheta;

  out->x = in->x*R[0][0] + in->y*R[0][1] + in->z*R[0][2];
  out->y = in->x*R[1][0] + in->y*R[1][1] + in->z*R[1][2];
  out->z = in->x*R[2][0] + in->y*R[2][1] + in->z*R[2][2];
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out)
{
  Axis3f rx;
  Axis3f ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

void sensorsBmi088Bmp388SetAccMode(accModes accMode)
{
  switch (accMode)
  {
    case ACC_MODE_PROPTEST:
      // bmi08a_soft_reset(&bmi088Dev);
      /* set bandwidth and range of accel (280Hz cut-off according to datasheet) */
      bmi088Dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
      bmi088Dev.accel_cfg.range = SENSORS_BMI088_ACCEL_FS_CFG;
      bmi088Dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;
      if (bmi08a_set_meas_conf(&bmi088Dev) != BMI08_OK)
      {
        DEBUG_PRINT("ACC config [FAIL]\n");
      }
      for (uint8_t i = 0; i < 3; i++)
      {
        lpf2pInit(&accLpf[i],  1000, 500);
      }
      break;
    case ACC_MODE_FLIGHT:
    default:
      /* set bandwidth and range of accel (145Hz cut-off according to datasheet) */
      bmi088Dev.accel_cfg.bw = BMI08_ACCEL_BW_OSR4;
      bmi088Dev.accel_cfg.range = SENSORS_BMI088_ACCEL_FS_CFG;
      bmi088Dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;
      if (bmi08a_set_meas_conf(&bmi088Dev) != BMI08_OK)
      {
        DEBUG_PRINT("ACC config [FAIL]\n");
      }
      for (uint8_t i = 0; i < 3; i++)
      {
        lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
      }
      break;
  }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
  }
}

void sensorsBmi088Bmp388DataAvailableCallback(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  imuIntTimestamp = usecTimestamp();
  xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

#ifdef GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES
LOG_GROUP_START(gyro)
LOG_ADD(LOG_INT16, xRaw, &gyroRaw.x)
LOG_ADD(LOG_INT16, yRaw, &gyroRaw.y)
LOG_ADD(LOG_INT16, zRaw, &gyroRaw.z)
LOG_ADD(LOG_FLOAT, xVariance, &gyroBiasRunning.variance.x)
LOG_ADD(LOG_FLOAT, yVariance, &gyroBiasRunning.variance.y)
LOG_ADD(LOG_FLOAT, zVariance, &gyroBiasRunning.variance.z)
LOG_GROUP_STOP(gyro)
#endif

PARAM_GROUP_START(imu_sensors)

/**
 * @brief Nonzero if BMP388 barometer is present
 */
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP388, &isBarometerPresent)

/**
 * @brief Euler angle Phi defining IMU orientation on the airframe (in degrees)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, imuPhi, &imuPhi)

/**
 * @brief Euler angle Theta defining IMU orientation on the airframe (in degrees)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, imuTheta, &imuTheta)

/**
 * @brief Euler angle Psi defining IMU orientation on the airframe (in degrees)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, imuPsi, &imuPsi)

PARAM_GROUP_STOP(imu_sensors)
