#pragma once

#include "IFilter.h"

namespace filters {

class KalmanFilter : public IFilter {
 public:
  explicit KalmanFilter(float process_noise = 0.01f, float sensor_noise = 0.25f,
                        float initial_error = 1.0f)
      : q_(process_noise), r_(sensor_noise), initial_error_(initial_error) {
    reset();
  }

  void init(float initial_value) override {
    x_ = initial_value;
    p_ = initial_error_;
    initialized_ = isfinite(initial_value);
  }
  float update(float measurement) override {
    if (!isfinite(measurement)) return x_;
    if (!initialized_) {
      init(measurement);
      return x_;
    }
    p_ += q_;
    const float gain = p_ / (p_ + r_);
    x_ += gain * (measurement - x_);
    p_ = (1.0f - gain) * p_;
    return x_;
  }
  const char* getName() const override { return "Kalman"; }
  void reset() override {
    x_ = NAN;
    p_ = initial_error_;
    initialized_ = false;
  }

 private:
  float q_;
  float r_;
  float initial_error_;
  float x_ = NAN;
  float p_ = 1.0f;
  bool initialized_ = false;
};

}  // namespace filters
