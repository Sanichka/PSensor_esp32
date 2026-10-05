#pragma once

#include <cmath>

class IFilter {
 public:
  virtual void init(float initial_value) = 0;
  virtual float update(float measurement) = 0;
  virtual const char* getName() const = 0;
  virtual void reset() = 0;
  virtual ~IFilter() = default;
};
