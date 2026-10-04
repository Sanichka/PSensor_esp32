#pragma once

#include <cmath>

struct KalmanFilter {
  float x;
  float p;
  float q;
  float r;
  float k;

  KalmanFilter(float process_noise = 0.01f, float sensor_noise = 0.25f,
               float init_est_error = 1.0f);

  float update(float measurement);
  void reset();
};

struct SensorData {
  float raw_hpa;
  float filtered_hpa;
  bool is_valid;
  const char* status;
};

SensorData processPressure(KalmanFilter& kf, float raw);
