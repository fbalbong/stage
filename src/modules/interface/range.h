/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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
 * range.h: Centralize range measurements for different directions
 *          and make them available as log
 */

#pragma once

typedef enum {
    rangeFront=0,
    rangeBack,
    rangeLeft,
    rangeRight,
    rangeUp,
    rangeDown,
    RANGE_T_END,
} rangeDirection_t;

/**
 * Set the range for a certain direction
 *
 * @param direction Direction of the range
 * @param range_m Distance to an object in meter
 */
void rangeSet(rangeDirection_t direction, float range_m);

/**
 * Get the range for a certain direction
 *
 * @param direction Direction of the range
 * @return Distance to an object in meter
 */
float rangeGet(rangeDirection_t direction);

/**
 * Enqueue a range measurement for distance to the ground in the current estimator.
 *
 * @param distance Distance to the ground (m)
 * @param stdDev The standard deviation of the range sample
 * @param timeStamp The time when the range was sampled (in sys ticks)
 */
void rangeEnqueueDownRangeInEstimator(float distance, float stdDev, uint32_t timeStamp);

/**
 * Enqueue a upward range measurement for distance to the roof in the current estimator.
 *
 * @param distance Distance to the roof (m)
 * @param stdDev The standard deviation of the range sample
 * @param timeStamp The time when the range was sampled (in sys ticks)
 */
void rangeEnqueueUpRangeInEstimator(float distance, float stdDev, uint32_t timeStamp);

/**
 * Enqueue a front range measurement for distance to the front wall in the current estimator.
 *
 * @param distance Distance to the front wall (m)
 * @param stdDev The standard deviation of the range sample
 * @param timeStamp The time when the range was sampled (in sys ticks)
 */
void rangeEnqueueFrontRangeInEstimator(float distance, float stdDev, uint32_t timeStamp);

/**
 * Enqueue a back range measurement for distance to the back wall in the current estimator.
 *
 * @param distance Distance to the back wall (m)
 * @param stdDev The standard deviation of the range sample
 * @param timeStamp The time when the range was sampled (in sys ticks)
 */
void rangeEnqueueBackRangeInEstimator(float distance, float stdDev, uint32_t timeStamp);

/**
 * Enqueue a left range measurement for distance to the left wall in the current estimator.
 *
 * @param distance Distance to the left wall (m)
 * @param stdDev The standard deviation of the range sample
 * @param timeStamp The time when the range was sampled (in sys ticks)
 */
void rangeEnqueueLeftRangeInEstimator(float distance, float stdDev, uint32_t timeStamp);

/**
 * Enqueue a right range measurement for distance to the right wall in the current estimator.
 *
 * @param distance Distance to the right wall (m)
 * @param stdDev The standard deviation of the range sample
 * @param timeStamp The time when the range was sampled (in sys ticks)
 */
void rangeEnqueueRightRangeInEstimator(float distance, float stdDev, uint32_t timeStamp);  


