#ifndef EKF_H
#define EKF_H

#include <Arduino.h>

struct EKFState {
  float x_mm;
  float y_mm;
  float theta_rad;
};

struct EKFTelemetry {
  float x_mm;
  float y_mm;
  float theta_deg;
};

static EKFState ekfState = {0.0f, 0.0f, 0.0f};

// Covariance matrix P
static float P[3][3];

// Tunable noise terms
static float EKF_Q_POS_BASE = 2.0f;         // mm^2 baseline process noise
static float EKF_Q_THETA_BASE = 0.0025f;    // rad^2 baseline process noise
static float EKF_R_YAW = 0.01f;             // rad^2 yaw measurement noise

// Robot constants: replace with your actual values
static float EKF_TICKS_PER_REV = 306.0f;
static float EKF_WHEEL_CIRCUMFERENCE_MM = 144.5f;
static float EKF_TRACK_WIDTH_MM = 72.0f;

inline float ekfWrapAngleRad(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

inline float ekfDegToRad(float deg) {
  return deg * 0.01745329252f;
}

inline float ekfRadToDeg(float rad) {
  return rad * 57.2957795f;
}

inline float ekfTicksToMM(float ticks) {
  return (ticks / EKF_TICKS_PER_REV) * EKF_WHEEL_CIRCUMFERENCE_MM;
}

void ekfConfigure(float ticksPerRev, float wheelCircumferenceMM, float trackWidthMM) {
  EKF_TICKS_PER_REV = ticksPerRev;
  EKF_WHEEL_CIRCUMFERENCE_MM = wheelCircumferenceMM;
  EKF_TRACK_WIDTH_MM = trackWidthMM;
}

void ekfInit(float x_mm, float y_mm, float theta_rad) {
  ekfState.x_mm = x_mm;
  ekfState.y_mm = y_mm;
  ekfState.theta_rad = theta_rad;

  P[0][0] = 25.0f; P[0][1] = 0.0f;  P[0][2] = 0.0f;
  P[1][0] = 0.0f;  P[1][1] = 25.0f; P[1][2] = 0.0f;
  P[2][0] = 0.0f;  P[2][1] = 0.0f;  P[2][2] = 0.05f;
}

EKFState ekfGetState() {
  return ekfState;
}

EKFTelemetry ekfGetTelemetry() {
  EKFTelemetry t;
  t.x_mm = ekfState.x_mm;
  t.y_mm = ekfState.y_mm;
  t.theta_deg = ekfRadToDeg(ekfState.theta_rad);
  return t;
}

void ekfPredict(long deltaLeftTicks, long deltaRightTicks) {
  float dL = ekfTicksToMM((float)deltaLeftTicks);
  float dR = ekfTicksToMM((float)deltaRightTicks);

  float ds = 0.5f * (dL + dR);
  float dTheta = (dR - dL) / EKF_TRACK_WIDTH_MM;

  float thetaMid = ekfState.theta_rad + 0.5f * dTheta;

  ekfState.x_mm += ds * cos(thetaMid);
  ekfState.y_mm += ds * sin(thetaMid);
  ekfState.theta_rad = ekfWrapAngleRad(ekfState.theta_rad + dTheta);

  float F[3][3] = {
    {1.0f, 0.0f, -ds * sin(thetaMid)},
    {0.0f, 1.0f,  ds * cos(thetaMid)},
    {0.0f, 0.0f, 1.0f}
  };

  float Q[3][3] = {
    {EKF_Q_POS_BASE + fabs(ds) * 0.05f, 0.0f, 0.0f},
    {0.0f, EKF_Q_POS_BASE + fabs(ds) * 0.05f, 0.0f},
    {0.0f, 0.0f, EKF_Q_THETA_BASE + fabs(dTheta) * 0.01f}
  };

  float FP[3][3] = {0};
  float FPFt[3][3] = {0};

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        FP[i][j] += F[i][k] * P[k][j];
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        FPFt[i][j] += FP[i][k] * F[j][k];
      }
      P[i][j] = FPFt[i][j] + Q[i][j];
    }
  }
}

void ekfUpdateYawDeg(float yaw_deg) {
  float z = ekfDegToRad(yaw_deg);
  float y = ekfWrapAngleRad(z - ekfState.theta_rad);

  float S = P[2][2] + EKF_R_YAW;

  float K[3];
  K[0] = P[0][2] / S;
  K[1] = P[1][2] / S;
  K[2] = P[2][2] / S;

  ekfState.x_mm += K[0] * y;
  ekfState.y_mm += K[1] * y;
  ekfState.theta_rad = ekfWrapAngleRad(ekfState.theta_rad + K[2] * y);

  float Pnew[3][3];
  for (int i = 0; i < 3; i++) {
    Pnew[i][0] = P[i][0];
    Pnew[i][1] = P[i][1];
    Pnew[i][2] = P[i][2];
  }

  for (int j = 0; j < 3; j++) {
    Pnew[0][j] -= K[0] * P[2][j];
    Pnew[1][j] -= K[1] * P[2][j];
    Pnew[2][j] -= K[2] * P[2][j];
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P[i][j] = Pnew[i][j];
    }
  }
}

#endif
