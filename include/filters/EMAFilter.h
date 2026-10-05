#pragma once

#include "IFilter.h"

namespace filters {

class EMAFilter : public IFilter {
 public:
  explicit EMAFilter(float alpha = 0.15f)
      : alpha_(alpha < 0.05f ? 0.05f : (alpha > 0.3f ? 0.3f : alpha)) {}

  void init(float initial_value) override {
    value_ = initial_value;
    initialized_ = isfinite(initial_value);
  }
  float update(float measurement) override {
    if (!isfinite(measurement)) return value_;
    if (!initialized_) {
      init(measurement);
    } else {
      value_ += alpha_ * (measurement - value_);
    }
    return value_;
  }
  const char* getName() const override { return "EMA"; }
  void reset() override {
    value_ = NAN;
    initialized_ = false;
  }

 private:
  float alpha_;
  float value_ = NAN;
  bool initialized_ = false;
};

}  // namespace filters
