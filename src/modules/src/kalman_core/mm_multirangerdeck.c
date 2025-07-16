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

#include "mm_multirangerdeck.h"
#include "param.h"

// Parameters for tuning the detection for f and r estimation in the Tof update.
// Factor multipled with the standard deviation of the measurement and compared to the prediction error (This is an int because of problems with param, if it is solved then it should probably be canged to a float) 
static int detection_factor = 10;
// The value the variance of f or r is set to when a detection happenes. It can probably be tuned to be smaller, but there it does not really seem to matter as long as it is "large enough"
static float variance_after_detection = 50; 
// Flag for turning the detection on and off. Without detection f and r tend to not change, causing the same problems as when not using them at all. There could be some way to get it to work without detection, but it is not implemented.
static bool use_detection = true;

// Parámetros comunes
#define MAX_TILT_DEG 30.0f
#define HALF_FOV_DEG (15.0f/2.0f)
#define MAX_TILT_RAD     (DEG_TO_RAD * MAX_TILT_DEG)
#define HALF_FOV_RAD     (DEG_TO_RAD * HALF_FOV_DEG)

void kalmanCoreUpdateWithBackTofUsingB(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // 1) Sacar tilt real del eje X_body
  float r20  = this->R[2][0];
  float tilt = asinf(r20);

  // 2) Sólo actualizar si tilt está dentro del rango
  if (fabsf(tilt) > MAX_TILT_RAD) {
    return;
  }

  // 3) Ajuste por FOV
  float angle = fabsf(tilt) - HALF_FOV_RAD;
  if (angle < 0.0f) {
    angle = 0.0f;
  }
  float cosA = cosf(angle);

  // 4) Medición y “hot init” de B
  float measured = tof->distance;
  if (!this->stateBInitialized) {
    this->S[KC_STATE_B] = this->S[KC_STATE_X] - measured * cosA;
    this->stateBInitialized = true;
  }

  // 5) Predicción y error
  float predicted = (this->S[KC_STATE_X] - this->S[KC_STATE_B]) / cosA;
  float error     = measured - predicted;

  // 6) Detección de outliers
  if (use_detection) {
    float thr = detection_factor * tof->stdDev;
    if (error*error > thr*thr) {
      this->P[KC_STATE_B][KC_STATE_B] = variance_after_detection;
      this->S[KC_STATE_B] = this->S[KC_STATE_X] - measured * cosA;
      error = 0.0f;
    }
  }

  // 7) Jacobiano
  h[KC_STATE_X] =  1.0f / cosA;
  h[KC_STATE_B] = -1.0f / cosA;

  // 8) Actualización
  kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
}

void kalmanCoreUpdateWithFrontTofUsingC(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // 1) Sacar tilt real del eje X_body
  float r20  = this->R[2][0];
  float tilt = asinf(r20);

  // 2) Sólo actualizar si tilt está dentro del rango
  if (fabsf(tilt) > MAX_TILT_RAD) {
    return;
  }

  // 3) Ajuste por FOV
  float angle = fabsf(tilt) - HALF_FOV_RAD;
  if (angle < 0.0f) {
    angle = 0.0f;
  }
  float cosA = cosf(angle);

  // 4) Medición y “hot init” de C
  float measured = tof->distance;
  if (!this->stateCInitialized) {
    this->S[KC_STATE_C]          = this->S[KC_STATE_X] + measured * cosA;
    this->stateCInitialized      = true;
  }

  // 5) Predicción y error
  float predicted = (this->S[KC_STATE_C] - this->S[KC_STATE_X]) / cosA;
  float error     = measured - predicted;

  // 6) Detección de outliers
  if (use_detection) {
    float thr = detection_factor * tof->stdDev;
    if (error*error > thr*thr) {
      this->P[KC_STATE_C][KC_STATE_C] = variance_after_detection;
      this->S[KC_STATE_C]             = this->S[KC_STATE_X] + measured * cosA;
      error = 0.0f;
    }
  }

  // 7) Jacobiano
  h[KC_STATE_X] = -1.0f / cosA;
  h[KC_STATE_C] =  1.0f / cosA;

  // 8) Actualización
  kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
}

void kalmanCoreUpdateWithRightTofUsingS(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // 1) Extraer roll (tilt lateral) real desde R[2][1]
  float r21  = this->R[2][1];
  float tilt = asinf(r21);

  // 2) Solo actualizar si el sensor está casi horizontal
  if (fabsf(tilt) > MAX_TILT_RAD) {
    return;
  }

  // 3) Descontar mitad del FOV y saturar a ≥ 0
  float angle = fabsf(tilt) - HALF_FOV_RAD;
  if (angle < 0.0f) angle = 0.0f;
  float cosA = cosf(angle);

  // 4) Medición y “hot init” de S
  float measured = tof->distance;
  if (!this->stateSInitialized) {
    this->S[KC_STATE_S]       = this->S[KC_STATE_Y] - measured * cosA;
    this->stateSInitialized   = true;
  }

  // 5) Predicción y error
  float predicted = (this->S[KC_STATE_Y] - this->S[KC_STATE_S]) / cosA;
  float error     = measured - predicted;

  // 6) Detección de outliers
  if (use_detection) {
    float thr = detection_factor * tof->stdDev;
    if (error*error > thr*thr) {
      this->P[KC_STATE_S][KC_STATE_S] = variance_after_detection;
      this->S[KC_STATE_S]             = this->S[KC_STATE_Y] - measured * cosA;
      error = 0.0f;
    }
  }

  // 7) Jacobiano
  h[KC_STATE_Y] =  1.0f / cosA;
  h[KC_STATE_S] = -1.0f / cosA;

  // 8) Actualización
  kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
}

void kalmanCoreUpdateWithLeftTofUsingT(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // 1) Extraer roll (tilt lateral) real desde R[2][1]
  float r21  = this->R[2][1];
  float tilt = asinf(r21);

  // 2) Solo actualizar si el sensor está casi horizontal
  if (fabsf(tilt) > MAX_TILT_RAD) {
    return;
  }

  // 3) Descontar mitad del FOV y saturar a ≥ 0
  float angle = fabsf(tilt) - HALF_FOV_RAD;
  if (angle < 0.0f) angle = 0.0f;
  float cosA = cosf(angle);

  // 4) Medición y “hot init” de T
  float measured = tof->distance;
  if (!this->stateTInitialized) {
    this->S[KC_STATE_T]       = this->S[KC_STATE_Y] + measured * cosA;
    this->stateTInitialized   = true;
  }

  // 5) Predicción y error
  float predicted = (this->S[KC_STATE_T] - this->S[KC_STATE_Y]) / cosA;
  float error     = measured - predicted;

  // 6) Detección de outliers
  if (use_detection) {
    float thr = detection_factor * tof->stdDev;
    if (error*error > thr*thr) {
      this->P[KC_STATE_T][KC_STATE_T] = variance_after_detection;
      this->S[KC_STATE_T]             = this->S[KC_STATE_Y] + measured * cosA;
      error = 0.0f;
    }
  }

  // 7) Jacobiano
  h[KC_STATE_Y] = -1.0f / cosA;
  h[KC_STATE_T] =  1.0f / cosA;

  // 8) Actualización
  kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
}

PARAM_GROUP_START(kalman)
/**
 * @brief The error threshold in Tof measurements that cause a detection of step in F or R
 */
 // PARAM_ADD_CORE(PARAM_UINT8, tofDetectionFactor, &detection_factor)
 /**
 * @brief The variance used in the covariance matrix when a detection happenes
 */
  //PARAM_ADD_CORE(PARAM_FLOAT, varianceAfterDetection, &variance_after_detection)
 /**
 * @brief Non zero to use detection in Tof measurements for changes in F or R
 */
  //PARAM_ADD_CORE(PARAM_UINT8, useDetection, &use_detection)

PARAM_GROUP_STOP(kalman)