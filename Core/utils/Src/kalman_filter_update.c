#include "kalman_filter_update.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof) {
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = { 0 };
  arm_matrix_instance_f32 H = { 1, KC_STATE_DIM, h };

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0) {
    // float angle = fabsf(acosf(this->R[2][2])) - radians(15.0f / 2.0f);
    // if (angle < 0.0f) {
    //   angle = 0.0f;
    // }
    //float predictedDistance = S[KC_STATE_Z] / cosf(angle);
    float predictedDistance = this->S[KC_STATE_Z] / this->R[2][2];
    float measuredDistance = tof->distance; // [m]

    //Measurement equation
    //
    // h = z/((R*z_b)\dot z_b) = z/cos(alpha)
    h[KC_STATE_Z] = 1.0 / this->R[2][2];
    //h[KC_STATE_Z] = 1 / cosf(angle);

    // Scalar update
    kalmanCoreScalarUpdate(this, &H, measuredDistance - predictedDistance, tof->stdDev);
  }
}