#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"
#include "log.h"
#include "bh1750.h"
#include "i2cdev.h"
#include <math.h>

#define BH1750_I2C_ADDRESS  0x23
#define BH1750_POWER_ON     0x01
#define BH1750_RESET        0x07
#define BH1750_CONT_H_RES   0x10 

static float bh1750lux = 0.0f;
I2C_Dev *dev = &deckBus;

bool bh1750_init(I2C_Dev *dev) {
    if (!i2cdevInit(dev)) {  
        DEBUG_PRINT("I2C init failed!\n");
        return false;
    }

    if (!i2cdevWriteByte(dev, BH1750_I2C_ADDRESS, 0, BH1750_POWER_ON)) { 
        DEBUG_PRINT("Failed to send POWER_ON command\n");
        return false;
    }

    if (!i2cdevWriteByte(dev, BH1750_I2C_ADDRESS, 0, BH1750_RESET)) { 
        DEBUG_PRINT("Failed to send RESET command\n");
        return false;
    }

    if (!i2cdevWriteByte(dev, BH1750_I2C_ADDRESS, 0, BH1750_CONT_H_RES)) {
        DEBUG_PRINT("Failed to set continuous high-res mode\n");
        return false;
    }

    DEBUG_PRINT("BH1750 initialized successfully\n");
    return true;
}

// Odczyt wartości natężenia światła w luksach
bool bh1750_read_lux(I2C_Dev *dev, float *lux) {
    uint8_t buffer[2] = {0};
    if (!i2cdevRead(dev, BH1750_I2C_ADDRESS, 2, buffer)) {
        DEBUG_PRINT("Failed to read data from BH1750\n");
        return false;
    }

    uint16_t raw = (buffer[0] << 8) | buffer[1];
    *lux = raw / 1.2f;

    return true;
}

static xQueueHandle sensorDataQueue;
STATIC_MEM_QUEUE_ALLOC(sensorDataQueue, 1, sizeof(float));

static void bh1750Task(void*);
STATIC_MEM_TASK_ALLOC(bh1750Task, BH1750_TASK_STACKSIZE);

static bool isInit = false;

void bh1750TaskInit() {
    sensorDataQueue = STATIC_MEM_QUEUE_CREATE(sensorDataQueue);
    STATIC_MEM_TASK_CREATE(bh1750Task, bh1750Task, BH1750_TASK_NAME, NULL, BH1750_TASK_PRI);
    isInit = true;
}

bool bh1750TaskTest() {
    return isInit;
}

void bh1750TaskEnqueueData(float value) {
    xQueueOverwrite(sensorDataQueue, &value);
}

static void bh1750Task(void* parameters) {
    bh1750_init(dev); // Inicjalizacja czujnika
    DEBUG_PRINT("BH1750 task is running!\n");

    while (true) {
        float lux = 0;
        if (bh1750_read_lux(dev, &lux)) {
            DEBUG_PRINT("Light intensity: %.2f lux\n", (double)lux);
        } else {
            DEBUG_PRINT("Failed to read light intensity\n");
        }        
        bh1750lux = lux;
        bh1750TaskEnqueueData(lux);
        vTaskDelay(pdMS_TO_TICKS(500)); // Odczyt co 0.5 s
    }
}


LOG_GROUP_START(bh1750)
LOG_ADD(LOG_FLOAT, lux, &bh1750lux)
LOG_GROUP_STOP(bh1750)