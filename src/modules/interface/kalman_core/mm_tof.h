/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
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

#include "kalman_core.h"

// Measurements of TOF from laser sensor
void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof);

// Kalman update using the TOF from downward laser sensor and F (floor height estimate)
void kalmanCoreUpdateWithTofUsingF(kalmanCoreData_t* this, tofMeasurement_t *tof);

// Kalman update using the TOF from upward laser sensor and R (roof height estimate)
void kalmanCoreUpdateWithUpTofUsingR(kalmanCoreData_t* this, tofMeasurement_t *tof);

// Kalman update using the TOF from backward laser sensor and B (back wall estimate)
void kalmanCoreUpdateWithBackTofUsingB(kalmanCoreData_t* this, tofMeasurement_t *tof);

// Kalman update using the TOF from front laser sensor and C (front wall estimate)
void kalmanCoreUpdateWithFrontTofUsingC(kalmanCoreData_t* this, tofMeasurement_t *tof);

// Kalman update using the TOF from right laser sensor and S (right wall estimate)
void kalmanCoreUpdateWithRightTofUsingS(kalmanCoreData_t* this, tofMeasurement_t *tof);

// Kalman update using the TOF from left laser sensor and T (left wall estimate)
void kalmanCoreUpdateWithLeftTofUsingT(kalmanCoreData_t* this, tofMeasurement_t *tof);