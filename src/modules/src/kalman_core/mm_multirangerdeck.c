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

void kalmanCoreUpdateWithBackTofUsingB(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[0][0]) > 0.1 && this->R[0][0] > 0){
    float angle = fabsf(acosf(this->R[0][0])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    
    float predictedDistance = (this->S[KC_STATE_X]-this->S[KC_STATE_B]) / cosf(angle);
    // float predictedDistance = (this->S[KC_STATE_Z]-this->S[KC_STATE_F]) / this->R[2][2];
    float measuredDistance = tof->distance; // [m]

    float error = measuredDistance-predictedDistance;


    if (use_detection){ 
      float threshold = detection_factor*(tof->stdDev);
      // If the error is very large it probably means that S[KC_STATE_F] needs to change
      if(error*error > threshold*threshold){
        // Give a best first guess of the new floor height and set the variance high
        this->P[KC_STATE_B][KC_STATE_B] = variance_after_detection;
        //this->S[KC_STATE_F] = this->S[KC_STATE_Z] - measuredDistance*this->R[2][2];
        this->S[KC_STATE_B] = this->S[KC_STATE_X] - measuredDistance*cosf(angle);

        error = 0.0f;
      }
    }

    //Measurement equation
    //
    // h = (z - f)/((R*z_b)\dot z_b) = z/cos(alpha)
    //h[KC_STATE_Z] = 1 / this->R[2][2];
    h[KC_STATE_X] = 1 / cosf(angle);

    //h[KC_STATE_F] = -1 / this->R[2][2];
    h[KC_STATE_B] = -1 / cosf(angle);


    // Scalar update
    kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
  }
}

void kalmanCoreUpdateWithFrontTofUsingC(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[0][0] -> 0)
  if (fabs(this->R[0][0]) > 0.1 && this->R[0][0] > 0){
    float angle = fabsf(acosf(this->R[0][0])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    
    float predictedDistance = (this->S[KC_STATE_C]-this->S[KC_STATE_X]) / cosf(angle);
    // float predictedDistance = (this->S[KC_STATE_Z]-this->S[KC_STATE_F]) / this->R[2][2];
    float measuredDistance = tof->distance; // [m]

    float error = measuredDistance-predictedDistance;

    if (use_detection){ 
      float threshold = detection_factor*(tof->stdDev);
      // If the error is very large it probably means that S[KC_STATE_F] needs to change
      if(error*error > threshold*threshold){
        // Give a best first guess of the new floor height and set the variance high
        this->P[KC_STATE_C][KC_STATE_C] = variance_after_detection;
        //this->S[KC_STATE_F] = this->S[KC_STATE_Z] - measuredDistance*this->R[2][2];
        this->S[KC_STATE_C] = this->S[KC_STATE_X] + measuredDistance*cosf(angle);

        error = 0.0f;
      }
    }
    //Measurement equation
    //
    // h = (r - z)/((R*z_b)\dot z_b) = z/cos(alpha)
    //h[KC_STATE_Z] = -1 / this->R[2][2];
    h[KC_STATE_X] = -1 / cosf(angle);

    //h[KC_STATE_R] = 1 / this->R[2][2];
    h[KC_STATE_C] = 1 / cosf(angle);

    // Scalar update
    kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
  }
}

void kalmanCoreUpdateWithRightTofUsingS(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[1][1] -> 0)
  if (fabs(this->R[1][1]) > 0.1 && this->R[1][1] > 0){
    float angle = fabsf(acosf(this->R[1][1])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    
    float predictedDistance = (this->S[KC_STATE_Y]-this->S[KC_STATE_S]) / cosf(angle);
    // float predictedDistance = (this->S[KC_STATE_Z]-this->S[KC_STATE_F]) / this->R[2][2];
    float measuredDistance = tof->distance; // [m]

    float error = measuredDistance-predictedDistance;

    if (use_detection){ 
      float threshold = detection_factor*(tof->stdDev);
      // If the error is very large it probably means that S[KC_STATE_F] needs to change
      if(error*error > threshold*threshold){
        // Give a best first guess of the new floor height and set the variance high
        this->P[KC_STATE_S][KC_STATE_S] = variance_after_detection;
        //this->S[KC_STATE_F] = this->S[KC_STATE_Z] - measuredDistance*this->R[2][2];
        this->S[KC_STATE_S] = this->S[KC_STATE_Y] - measuredDistance*cosf(angle);

        error = 0.0f;
      }
    }

    //Measurement equation
    //
    // h = (z - f)/((R*z_b)\dot z_b) = z/cos(alpha)
    //h[KC_STATE_Z] = 1 / this->R[2][2];
    h[KC_STATE_Y] = 1 / cosf(angle);

    //h[KC_STATE_F] = -1 / this->R[2][2];
    h[KC_STATE_S] = -1 / cosf(angle);
    // Scalar update
    kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
  }
}

void kalmanCoreUpdateWithLeftTofUsingT(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[1][1] -> 0)
  if (fabs(this->R[1][1]) > 0.1 && this->R[1][1] > 0){
    float angle = fabsf(acosf(this->R[1][1])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    
    float predictedDistance = (this->S[KC_STATE_T]-this->S[KC_STATE_Y]) / cosf(angle);
    // float predictedDistance = (this->S[KC_STATE_Z]-this->S[KC_STATE_F]) / this->R[2][2];
    float measuredDistance = tof->distance; // [m]

    float error = measuredDistance-predictedDistance;

    if (use_detection){ 
      float threshold = detection_factor*(tof->stdDev);
      // If the error is very large it probably means that S[KC_STATE_F] needs to change
      if(error*error > threshold*threshold){
        // Give a best first guess of the new floor height and set the variance high
        this->P[KC_STATE_T][KC_STATE_T] = variance_after_detection;
        //this->S[KC_STATE_F] = this->S[KC_STATE_Z] - measuredDistance*this->R[2][2];
        this->S[KC_STATE_T] = this->S[KC_STATE_Y] + measuredDistance*cosf(angle);

        error = 0.0f;
      }
    }

    //Measurement equation
    //
    // h = (z - f)/((R*z_b)\dot z_b) = z/cos(alpha)
    //h[KC_STATE_Z] = 1 / this->R[2][2];
    h[KC_STATE_Y] = -1 / cosf(angle);

    //h[KC_STATE_F] = -1 / this->R[2][2];
    h[KC_STATE_T] = 1 / cosf(angle);
    // Scalar update
    kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
  }
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