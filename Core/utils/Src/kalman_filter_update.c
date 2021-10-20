#include "kalman_filter_update.h"
#include "debug.h"
#include "cal.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof) {
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = { 0 };
  arm_matrix_instance_f32 H = { 1, KC_STATE_DIM, h };

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0) {
    float angle = fabsf(acosf(this->R[2][2])) - radians(15.0f / 2.0f);
    if (angle < 0.0f)
      angle = 0.0f;
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

static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro) {
  // Inclusion of flow measurements in the EKF done by two scalar updates
  // ~~~ Camera constants ~~~
  // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
  float pixelNbr = 30.0;                      // [pixels] (same in x and y)
  //float thetapix = DEG_TO_RAD * 4.0f;   // [rad]    (same in x and y)
  float thetapix = radians(4.2f);
  //~~~ Body rates ~~~
  // TODO check if this is feasible or if some filtering has to be done
  float omegax_b = radians(gyro->x);
  float omegay_b = radians(gyro->y);

  // ~~~ Moves the body velocity into the global coordinate system ~~~
  // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
  //
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  //
  // where \hat{} denotes a basis vector, \dot{} denotes a derivative and
  // _G and _B refer to the global/body coordinate systems.

  // Modification 1
  //dx_g = R[0][0] * S[KC_STATE_PX] + R[0][1] * S[KC_STATE_PY] + R[0][2] * S[KC_STATE_PZ];
  //dy_g = R[1][0] * S[KC_STATE_PX] + R[1][1] * S[KC_STATE_PY] + R[1][2] * S[KC_STATE_PZ];
  float dx_g = this->S[KC_STATE_PX];
  float dy_g = this->S[KC_STATE_PY];
  float z_g;
  // Saturate elevation in prediction and correction to avoid singularities
  if (this->S[KC_STATE_Z] < 0.1f)
    z_g = 0.1;
  else
    z_g = this->S[KC_STATE_Z];

  // ~~~ X velocity prediction and update ~~~
  // predics the number of accumulated pixels in the x-direction
  float omegaFactor = 1.25f;
  float hx[KC_STATE_DIM] = { 0 };
  arm_matrix_instance_f32 Hx = { 1, KC_STATE_DIM, hx };
  predictedNX = (flow->dt * pixelNbr / thetapix) * ((dx_g * this->R[2][2] / z_g) - omegaFactor * omegay_b);
  measuredNX = flow->dpixelx;

  // derive measurement equation with respect to dx (and z?)
  hx[KC_STATE_Z] = (pixelNbr * flow->dt / thetapix) * ((this->R[2][2] * dx_g) / (-z_g * z_g));
  hx[KC_STATE_PX] = (pixelNbr * flow->dt / thetapix) * (this->R[2][2] / z_g);

  /*! X update */
  kalmanCoreScalarUpdate(this, &Hx, measuredNX - predictedNX, flow->stdDevX);

  // ~~~ Y velocity prediction and update ~~~
  float hy[KC_STATE_DIM] = { 0 };
  arm_matrix_instance_f32 Hy = { 1, KC_STATE_DIM, hy };
  predictedNY = (flow->dt * pixelNbr / thetapix) * ((dy_g * this->R[2][2] / z_g) + omegaFactor * omegax_b);
  measuredNY = flow->dpixely;

  // derive measurement equation with respect to dy (and z?)
  hy[KC_STATE_Z] = (pixelNbr * flow->dt / thetapix) * ((this->R[2][2] * dy_g) / (-z_g * z_g));
  hy[KC_STATE_PY] = (pixelNbr * flow->dt / thetapix) * (this->R[2][2] / z_g);

  /*! Y update */
  kalmanCoreScalarUpdate(this, &Hy, measuredNY - predictedNY, flow->stdDevY);
}

static float measNoiseBaro = 2.0f; // meters
void kalmanCoreUpdateWithBaro(kalmanCoreData_t* this, float baroAsl, bool quadIsFlying) {
  return;
  float h[KC_STATE_DIM] = { 0 };
  arm_matrix_instance_f32 H = { 1, KC_STATE_DIM, h };

  h[KC_STATE_Z] = 1;

  if (!quadIsFlying || this->baroReferenceHeight < 1)
    this->baroReferenceHeight = baroAsl;

  float meas = (baroAsl - this->baroReferenceHeight);
  kalmanCoreScalarUpdate(this, &H, meas - this->S[KC_STATE_Z], measNoiseBaro);
}