#include "PressureProcessing.h"

KalmanFilter::KalmanFilter(float process_noise, float sensor_noise,
                           float init_est_error)
    : x(NAN),
      p(init_est_error),
      q(process_noise),
      r(sensor_noise),
      k(0.0f) {}

float KalmanFilter::update(float measurement) {
  if (std::isnan(x)) {
    x = measurement;
    return x;
  }
  p += q;
  k = p / (p + r);
  x += k * (measurement - x);
  p = (1.0f - k) * p;
  return x;
}

void KalmanFilter::reset() {
  x = NAN;
  p = 1.0f;
}

SensorData processPressure(KalmanFilter& kf, float raw) {
  SensorData data = {raw, NAN, false, "ERR_UNKNOWN"};
  if (std::isnan(raw) || raw <= 0.0f) {
    data.status = "ERR_DISCONNECTED";
    return data;
  }
  if (raw < 300.0f || raw > 1100.0f) {
    data.status = "ERR_OUT_OF_RANGE";
    return data;
  }
  data.status = (!std::isnan(kf.x) && std::fabs(raw - kf.x) > 4.0f)
                    ? "WARN_SPIKE"
                    : "OK";
  data.filtered_hpa = kf.update(raw);
  data.is_valid = true;
  return data;
}

SensorData processPressure(IFilter& filter, float raw) {
  SensorData data = {raw, NAN, false, "ERR_UNKNOWN"};
  if (std::isnan(raw) || raw <= 0.0f) {
    data.status = "ERR_DISCONNECTED";
    return data;
  }
  if (raw < 300.0f || raw > 1100.0f) {
    data.status = "ERR_OUT_OF_RANGE";
    return data;
  }
  data.status = "OK";
  data.filtered_hpa = filter.update(raw);
  data.is_valid = true;
  return data;
}
