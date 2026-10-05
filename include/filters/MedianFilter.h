#pragma once

#include <cstddef>
#include "IFilter.h"

namespace filters {

template <size_t Capacity>
class MedianFilter : public IFilter {
  static_assert(Capacity > 0, "Median capacity must be positive");

 public:
  explicit MedianFilter(size_t window_size = Capacity)
      : window_size_(window_size > Capacity ? Capacity : window_size) {
    reset();
  }

  void init(float initial_value) override {
    reset();
    if (!isfinite(initial_value)) return;
    for (size_t i = 0; i < window_size_; ++i) values_[i] = initial_value;
    count_ = window_size_;
  }

  float update(float measurement) override {
    if (isfinite(measurement)) {
      values_[index_] = measurement;
      if (count_ < window_size_) ++count_;
      index_ = (index_ + 1) % window_size_;
    }
    if (count_ == 0) return NAN;
    float sorted[Capacity];
    for (size_t i = 0; i < count_; ++i) sorted[i] = values_[i];
    for (size_t i = 1; i < count_; ++i) {
      float value = sorted[i];
      size_t j = i;
      while (j > 0 && sorted[j - 1] > value) {
        sorted[j] = sorted[j - 1];
        --j;
      }
      sorted[j] = value;
    }
    return sorted[count_ / 2];
  }

  const char* getName() const override { return "Median"; }
  void reset() override {
    for (size_t i = 0; i < Capacity; ++i) values_[i] = 0.0f;
    index_ = 0;
    count_ = 0;
  }

 private:
  float values_[Capacity];
  size_t window_size_;
  size_t index_ = 0;
  size_t count_ = 0;
};

}  // namespace filters
