#pragma once

#include "IFilter.h"

namespace filters {

class ComplementaryFusion : public IFilter {
 public:
  explicit ComplementaryFusion(float accelerometer_weight = 0.02f,
                               float meters_per_hpa = 8.3f)
      : accel_weight_(accelerometer_weight < 0.0f
                          ? 0.0f
                          : (accelerometer_weight > 1.0f ? 1.0f
                                                         : accelerometer_weight)),
        meters_per_hpa_(meters_per_hpa) {
    reset();
  }

  void init(float initial_value) override {
    pressure_altitude_ = initial_value;
    fused_altitude_ = initial_value;
    velocity_ = 0.0f;
    initialized_ = isfinite(initial_value);
  }

  float update(float measurement) override {
    return update(measurement, 0.0f, 0.01f);
  }

  float update(float pressure_altitude, float az, float dt) {
    if (!isfinite(pressure_altitude) || !isfinite(az) || dt <= 0.0f) {
      return fused_altitude_;
    }
    if (!initialized_) {
      init(pressure_altitude);
      return fused_altitude_;
    }
    velocity_ += az * dt;
    const float inertial_altitude = fused_altitude_ + velocity_ * dt;
    pressure_altitude_ = pressure_altitude;
    fused_altitude_ = (1.0f - accel_weight_) * pressure_altitude_ +
                      accel_weight_ * inertial_altitude;
    return fused_altitude_;
  }

  const char* getName() const override { return "Complementary"; }
  void reset() override {
    pressure_altitude_ = NAN;
    fused_altitude_ = NAN;
    velocity_ = 0.0f;
    initialized_ = false;
  }

 private:
  float accel_weight_;
  float meters_per_hpa_;
  float pressure_altitude_ = NAN;
  float fused_altitude_ = NAN;
  float velocity_ = 0.0f;
  bool initialized_ = false;
};

}  // namespace filters
