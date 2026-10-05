#pragma once

#include <cstddef>
#include "IFilter.h"

namespace filters {

template <size_t Capacity>
class SMAFilter : public IFilter {
  static_assert(Capacity > 0, "SMA capacity must be positive");

 public:
  explicit SMAFilter(size_t window_size = Capacity)
      : window_size_(window_size > Capacity ? Capacity : window_size) {
    reset();
  }

  void init(float initial_value) override {
    reset();
    if (!isfinite(initial_value)) return;
    for (size_t i = 0; i < window_size_; ++i) values_[i] = initial_value;
    sum_ = initial_value * static_cast<float>(window_size_);
    count_ = window_size_;
  }

  float update(float measurement) override {
    if (!isfinite(measurement)) return count_ == 0 ? NAN : sum_ / count_;
    if (count_ < window_size_) {
      values_[index_] = measurement;
      sum_ += measurement;
      ++count_;
    } else {
      sum_ += measurement - values_[index_];
      values_[index_] = measurement;
    }
    index_ = (index_ + 1) % window_size_;
    return sum_ / static_cast<float>(count_);
  }

  const char* getName() const override { return "SMA"; }
  void reset() override {
    for (size_t i = 0; i < Capacity; ++i) values_[i] = 0.0f;
    index_ = 0;
    count_ = 0;
    sum_ = 0.0f;
  }

 private:
  float values_[Capacity];
  size_t window_size_;
  size_t index_ = 0;
  size_t count_ = 0;
  float sum_ = 0.0f;
};

}  // namespace filters
