#pragma once

#include <cmath>
#include "IFilter.h"

namespace filters {

class NotchFilter : public IFilter {
 public:
  explicit NotchFilter(float sample_rate_hz = 100.0f, float notch_hz = 10.0f,
                       float radius = 0.95f)
      : sample_rate_hz_(sample_rate_hz <= 0.0f ? 100.0f : sample_rate_hz),
        notch_hz_(notch_hz),
        radius_(radius < 0.0f ? 0.0f : (radius > 0.999f ? 0.999f : radius)) {
    reset();
  }

  void init(float initial_value) override {
    reset();
    x1_ = x2_ = y1_ = y2_ = initial_value;
    initialized_ = isfinite(initial_value);
  }
  float update(float measurement) override {
    if (!isfinite(measurement)) return y1_;
    if (!initialized_) {
      init(measurement);
      return measurement;
    }
    const float omega = 2.0f * 3.14159265358979323846f * notch_hz_ /
                        sample_rate_hz_;
    const float cosine = cosf(omega);
    const float b0 = 1.0f;
    const float b1 = -2.0f * cosine;
    const float b2 = 1.0f;
    const float a1 = -2.0f * radius_ * cosine;
    const float a2 = radius_ * radius_;
    const float dc_gain = (1.0f + a1 + a2) / (b0 + b1 + b2);
    const float output = dc_gain * (b0 * measurement + b1 * x1_ + b2 * x2_) -
                         a1 * y1_ - a2 * y2_;
    x2_ = x1_;
    x1_ = measurement;
    y2_ = y1_;
    y1_ = output;
    return output;
  }
  const char* getName() const override { return "Notch"; }
  void reset() override {
    x1_ = x2_ = y1_ = y2_ = NAN;
    initialized_ = false;
  }

 private:
  float sample_rate_hz_;
  float notch_hz_;
  float radius_;
  float x1_ = NAN, x2_ = NAN, y1_ = NAN, y2_ = NAN;
  bool initialized_ = false;
};

}  // namespace filters
