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
static int detection_factor = 15;
// The value the variance of f or r is set to when a detection happenes. It can probably be tuned to be smaller, but there it does not really seem to matter as long as it is "large enough"
static float variance_after_detection = 35; 
// Flag for turning the detection on and off. Without detection f and r tend to not change, causing the same problems as when not using them at all. There could be some way to get it to work without detection, but it is not implemented.
static bool use_detection = true;

void kalmanCoreUpdateWithBackTofUsingB(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // 1. Obtener componentes de inclinación vertical
  const float r20 = this->R[2][0];
  const float sin_alpha = fabsf(r20);
  
  // 2. Calcular coseno efectivo (con límite por FOV)
  const float HALF_FOV = DEG_TO_RAD * (15.0f / 2.0f);
  const float base_cos = sqrtf(1 - sin_alpha * sin_alpha);
  const float min_cos = cosf(HALF_FOV);
  const float effective_cos = (base_cos > min_cos) ? base_cos : min_cos;

  // 3. Validar confiabilidad de la medición
  if (effective_cos < 0.1f) return;

  // 4. Predicción y error (pared trasera: X_drone > B_wall)
  const float predicted = (this->S[KC_STATE_X] - this->S[KC_STATE_B]) / effective_cos;
  const float error = tof->distance - predicted;

  // 5. Detección de outliers (reset de B)
  if (use_detection && fabsf(error) > detection_factor * tof->stdDev) {
    this->P[KC_STATE_B][KC_STATE_B] = variance_after_detection;
    this->S[KC_STATE_B] = this->S[KC_STATE_X] - tof->distance * effective_cos;
    return;  // Saltar actualización esta iteración
  }

  // 6. Jacobiano
  h[KC_STATE_X] =  1.0f / effective_cos;
  h[KC_STATE_B] = -1.0f / effective_cos;

  // 7. Actualización
  kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
}

void kalmanCoreUpdateWithFrontTofUsingC(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // 1. Obtener componentes de inclinación vertical
  const float r20 = this->R[2][0];
  const float sin_alpha = fabsf(r20);
  
  // 2. Calcular coseno efectivo
  const float HALF_FOV = DEG_TO_RAD * (15.0f / 2.0f);
  const float base_cos = sqrtf(1 - sin_alpha * sin_alpha);
  const float min_cos = cosf(HALF_FOV);
  const float effective_cos = (base_cos > min_cos) ? base_cos : min_cos;

  // 3. Validar confiabilidad
  if (effective_cos < 0.1f) return;

  // 4. Predicción y error (pared frontal: C_wall > X_drone)
  const float predicted = (this->S[KC_STATE_C] - this->S[KC_STATE_X]) / effective_cos;
  const float error = tof->distance - predicted;

  // 5. Detección de outliers (reset de C)
  if (use_detection && fabsf(error) > detection_factor * tof->stdDev) {
    this->P[KC_STATE_C][KC_STATE_C] = variance_after_detection;
    this->S[KC_STATE_C] = this->S[KC_STATE_X] + tof->distance * effective_cos;
    return;
  }

  // 6. Jacobiano
  h[KC_STATE_X] = -1.0f / effective_cos;
  h[KC_STATE_C] =  1.0f / effective_cos;

  // 7. Actualización
  kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
}

void kalmanCoreUpdateWithRightTofUsingS(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // 1. Obtener componente vertical del eje Y
  const float r21 = this->R[2][1];
  const float sin_alpha = fabsf(r21);
  
  // 2. Calcular coseno efectivo (con límite por FOV)
  const float HALF_FOV = DEG_TO_RAD * (15.0f / 2.0f);
  const float base_cos = sqrtf(1 - sin_alpha * sin_alpha);
  const float min_cos = cosf(HALF_FOV);
  const float effective_cos = (base_cos > min_cos) ? base_cos : min_cos;

  // 3. Validar confiabilidad de la medición
  if (effective_cos < 0.1f) return;

  // 4. Predicción y error (pared derecha: Y_drone > S_wall)
  const float predicted = (this->S[KC_STATE_Y] - this->S[KC_STATE_S]) / effective_cos;
  const float error = tof->distance - predicted;

  // 5. Detección de outliers (reset de S)
  if (use_detection && fabsf(error) > detection_factor * tof->stdDev) {
    this->P[KC_STATE_S][KC_STATE_S] = variance_after_detection;
    this->S[KC_STATE_S] = this->S[KC_STATE_Y] - tof->distance * effective_cos;
    return;  // Saltar actualización esta iteración
  }

  // 6. Jacobiano
  h[KC_STATE_Y] =  1.0f / effective_cos;
  h[KC_STATE_S] = -1.0f / effective_cos;

  // 7. Actualización
  kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
}

void kalmanCoreUpdateWithLeftTofUsingT(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // 1. Obtener componente vertical del eje Y
  const float r21 = this->R[2][1];
  const float sin_alpha = fabsf(r21);
  
  // 2. Calcular coseno efectivo
  const float HALF_FOV = DEG_TO_RAD * (15.0f / 2.0f);
  const float base_cos = sqrtf(1 - sin_alpha * sin_alpha);
  const float min_cos = cosf(HALF_FOV);
  const float effective_cos = (base_cos > min_cos) ? base_cos : min_cos;

  // 3. Validar confiabilidad
  if (effective_cos < 0.1f) return;

  // 4. Predicción y error (pared izquierda: T_wall > Y_drone)
  const float predicted = (this->S[KC_STATE_T] - this->S[KC_STATE_Y]) / effective_cos;
  const float error = tof->distance - predicted;

  // 5. Detección de outliers (reset de T)
  if (use_detection && fabsf(error) > detection_factor * tof->stdDev) {
    this->P[KC_STATE_T][KC_STATE_T] = variance_after_detection;
    this->S[KC_STATE_T] = this->S[KC_STATE_Y] + tof->distance * effective_cos;
    return;
  }

  // 6. Jacobiano
  h[KC_STATE_Y] = -1.0f / effective_cos;
  h[KC_STATE_T] =  1.0f / effective_cos;

  // 7. Actualización
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