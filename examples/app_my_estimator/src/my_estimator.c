/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "MYESTIMATOR"
#include "debug.h"


void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  
  }
}

// The new estimator goes here --------------------------------------------
#include "estimator.h"

// Call the Kalman estimator in this example to make it possible to fly. When you implement you own estimator, there is
// no need to include the Kalman estimator.
#include "estimator_kalman.h"
void estimatorOutOfTreeInit() {
  // Initialize your estimator data here...
  DEBUG_PRINT("Out of tree estimator initialized\n");

  // Initialize the Kalman estimator
  estimatorKalmanInit();
  DEBUG_PRINT("Kalman estimator initialized\n");
}

bool estimatorOutOfTreeTest() {
  // Test your estimator here, return true if it is working
  DEBUG_PRINT("Out of tree estimator test\n");
  return true;
}

void estimatorOutOfTree(state_t *state, const stabilizerStep_t stabilizerStep) {
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The dataMutex must only be locked short periods by the task.

  DEBUG_PRINT("Out of tree estimator update\n");

  // Call the Kalman estimator instead in this example to make it possible to fly
  estimatorKalman(state, stabilizerStep);
}