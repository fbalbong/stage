/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2021, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* multiranger.c: Multiranger deck driver */
#include "deck.h"
#include "param.h"

#define DEBUG_MODULE "MR"

#include "system.h"
#include "debug.h"
#include "log.h"
#include "pca95x4.h"
#include "vl53l1x.h"
#include "range.h"
#include "static_mem.h"

#include "i2cdev.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

#include "cf_math.h"

static bool isInit = false;
static bool isTested = false;
static bool isPassed = false;
static uint16_t filterMask = 1 << VL53L1_RANGESTATUS_RANGE_VALID;

#define MR_PIN_UP PCA95X4_P0
#define MR_PIN_FRONT PCA95X4_P4
#define MR_PIN_BACK PCA95X4_P1
#define MR_PIN_LEFT PCA95X4_P6
#define MR_PIN_RIGHT PCA95X4_P2

#define RANGE_UP_OUTLIER_LIMIT 4000 // the measured range is in [mm]

NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devFront;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devBack;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devUp;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devLeft;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devRight;

// Measurement noise model
static const float expPointA = 2.5f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 4.0f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

static bool mrInitSensor(VL53L1_Dev_t *pdev, uint32_t pca95pin, char *name)
{
    bool status;

    // Bring up VL53 by releasing XSHUT
    pca95x4SetOutput(PCA95X4_DEFAULT_ADDRESS, pca95pin);
    // Let VL53 boot
    vTaskDelay(M2T(2));
    // Init VL53
    if (vl53l1xInit(pdev, I2C1_DEV))
    {
        DEBUG_PRINT("Init %s sensor [OK]\n", name);
        status = true;
    }
    else
    {
        DEBUG_PRINT("Init %s sensor [FAIL]\n", name);
        status = false;
    }

    return status;

    // pre-compute constant in the measurement noise model for kalman
    expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);
}

static uint16_t mrGetMeasurementAndRestart(VL53L1_Dev_t *dev)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range = 0;

    while (dataReady == 0)
    {
        status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }

    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);
    range = rangingData.RangeMilliMeter; // Sin filtro RangeStatus ni 32767

    VL53L1_StopMeasurement(dev);
    status = VL53L1_StartMeasurement(dev);

    return range;
}

static void mrTask(void *param)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;

    systemWaitStart();

    // Restart all sensors
    status = VL53L1_StopMeasurement(&devFront);
    status = VL53L1_StartMeasurement(&devFront);
    status = VL53L1_StopMeasurement(&devBack);
    status = VL53L1_StartMeasurement(&devBack);
    status = VL53L1_StopMeasurement(&devUp);
    status = VL53L1_StartMeasurement(&devUp);
    status = VL53L1_StopMeasurement(&devLeft);
    status = VL53L1_StartMeasurement(&devLeft);
    status = VL53L1_StopMeasurement(&devRight);
    status = VL53L1_StartMeasurement(&devRight);
    status = status;

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
    vTaskDelayUntil(&lastWakeTime, M2T(25));

    // Obtén todas las medidas crudas en mm
    uint16_t rawUp    = mrGetMeasurementAndRestart(&devUp);
    uint16_t rawFront = mrGetMeasurementAndRestart(&devFront);
    uint16_t rawBack  = mrGetMeasurementAndRestart(&devBack);
    uint16_t rawLeft  = mrGetMeasurementAndRestart(&devLeft);
    uint16_t rawRight = mrGetMeasurementAndRestart(&devRight);

    // Loggea solo si la medida es < 5m
    if (rawUp    < RANGE_UP_OUTLIER_LIMIT)    rangeSet(rangeUp,    rawUp    / 1000.0f);
    if (rawFront < RANGE_UP_OUTLIER_LIMIT)    rangeSet(rangeFront, rawFront / 1000.0f);
    if (rawBack  < RANGE_UP_OUTLIER_LIMIT)    rangeSet(rangeBack,  rawBack  / 1000.0f);
    if (rawLeft  < RANGE_UP_OUTLIER_LIMIT)    rangeSet(rangeLeft,  rawLeft  / 1000.0f);
    if (rawRight < RANGE_UP_OUTLIER_LIMIT)    rangeSet(rangeRight, rawRight / 1000.0f);

    // Add up range to kalman filter measurements (solo si es válido)
    if (rawUp < RANGE_UP_OUTLIER_LIMIT) {
        float distance = (float)rawUp * 0.001f;
        float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
        rangeEnqueueUpRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
    if (rawFront < RANGE_UP_OUTLIER_LIMIT) {
        float distance = (float)rawFront * 0.001f;
        float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
        rangeEnqueueFrontRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
    if (rawBack < RANGE_UP_OUTLIER_LIMIT) {
        float distance = (float)rawBack * 0.001f;
        float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
        rangeEnqueueBackRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
    if (rawLeft < RANGE_UP_OUTLIER_LIMIT) {
        float distance = (float)rawLeft * 0.001f;
        float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
        rangeEnqueueLeftRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
    if (rawRight < RANGE_UP_OUTLIER_LIMIT) {
        float distance = (float)rawRight * 0.001f;
        float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
        rangeEnqueueRightRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
    }
    
    

}

static void mrInit()
{
    if (isInit)
    {
        return;
    }

    pca95x4Init();

    pca95x4ConfigOutput(PCA95X4_DEFAULT_ADDRESS,
                        ~(MR_PIN_UP |
                          MR_PIN_RIGHT |
                          MR_PIN_LEFT |
                          MR_PIN_FRONT |
                          MR_PIN_BACK));

    pca95x4ClearOutput(PCA95X4_DEFAULT_ADDRESS,
                       MR_PIN_UP |
                       MR_PIN_RIGHT |
                       MR_PIN_LEFT |
                       MR_PIN_FRONT |
                       MR_PIN_BACK);

    isInit = true;

    xTaskCreate(mrTask, MULTIRANGER_TASK_NAME, MULTIRANGER_TASK_STACKSIZE, NULL,
                MULTIRANGER_TASK_PRI, NULL);
}

static bool mrTest()
{
    if (isTested)
    {
        return isPassed;
    }

    isPassed = isInit;

    isPassed &= mrInitSensor(&devFront, MR_PIN_FRONT, "front");
    isPassed &= mrInitSensor(&devBack, MR_PIN_BACK, "back");
    isPassed &= mrInitSensor(&devUp, MR_PIN_UP, "up");
    isPassed &= mrInitSensor(&devLeft, MR_PIN_LEFT, "left");
    isPassed &= mrInitSensor(&devRight, MR_PIN_RIGHT, "right");

    isTested = true;

    return isPassed;
}

static const DeckDriver multiranger_deck = {
    .vid = 0xBC,
    .pid = 0x0C,
    .name = "bcMultiranger",

    .usedGpio = 0,
    .usedPeriph = DECK_USING_I2C,

    .init = mrInit,
    .test = mrTest,
};

DECK_DRIVER(multiranger_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Multi-ranger deck](%https://store.bitcraze.io/collections/decks/products/multi-ranger-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &isInit)

PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(multiranger)
/**
 * @brief Filter mask determining which range measurements is to be let through based on the range status of the VL53L1 chip
 */
PARAM_ADD(PARAM_UINT16, filterMask, &filterMask)

PARAM_GROUP_STOP(multiranger)
