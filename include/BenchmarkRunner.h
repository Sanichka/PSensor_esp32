#pragma once

#include <cmath>
#include <cstddef>
#include <cstdio>
#ifndef ARDUINO
#include <chrono>
#endif

#include "filters/IFilter.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

struct FilterBenchmarkResult {
  const char* name;
  size_t object_bytes;
  float average_us;
  float peak_us;
  float variance;
  float standard_deviation;
  float impulse_deviation;
  int step_lag_iterations;
  bool allocation_safe;
};

class FilterBenchmark {
 public:
  static constexpr size_t kSamples = 100;
  static constexpr size_t kMaxFilters = 16;
  static constexpr float kBaseline = 1000.0f;

  static uint64_t nowMicros() {
#ifdef ARDUINO
    return static_cast<uint64_t>(esp_timer_get_time());
#else
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
#endif
  }

  static FilterBenchmarkResult run(IFilter& filter, size_t object_bytes) {
    FilterBenchmarkResult result = {filter.getName(), object_bytes, 0.0f,
                                    0.0f, 0.0f, 0.0f, 0.0f, -1, true};
    filter.reset();

    float outputs[kSamples];
    size_t total_us = 0;
    for (size_t i = 0; i < kSamples; ++i) {
      const float noise = static_cast<float>((static_cast<int>(i * 37) % 21) - 10) *
                          0.02f;
      const float input = kBaseline + noise;
      const uint64_t start = nowMicros();
      outputs[i] = filter.update(input);
      const uint64_t elapsed = nowMicros() - start;
      total_us += elapsed;
      if (static_cast<float>(elapsed) > result.peak_us) {
        result.peak_us = static_cast<float>(elapsed);
      }
    }
    result.average_us = static_cast<float>(total_us) / kSamples;

    float sum = 0.0f;
    for (float output : outputs) sum += output;
    const float mean = sum / kSamples;
    for (float output : outputs) {
      const float delta = output - mean;
      result.variance += delta * delta;
    }
    result.variance /= kSamples;
    result.standard_deviation = sqrtf(result.variance);

    filter.reset();
    filter.init(kBaseline);
    const float impulse_output = filter.update(kBaseline + 15.0f);
    result.impulse_deviation = fabsf(impulse_output - kBaseline);

    filter.reset();
    filter.init(kBaseline);
    const float target = kBaseline + 10.0f;
    for (int i = 1; i <= 100; ++i) {
      if (filter.update(target) >= kBaseline + 9.0f) {
        result.step_lag_iterations = i;
        break;
      }
    }
    return result;
  }

  static void printMarkdown(IFilter* const filters[],
                            const size_t object_sizes[], size_t count) {
    if (count > kMaxFilters) {
#ifdef ARDUINO
      Serial.println("Filter benchmark error: too many filters.");
#else
      printf("Filter benchmark error: too many filters.\n");
#endif
      return;
    }
#ifdef ARDUINO
    Serial.println();
    Serial.println("================================");
    Serial.println("         FILTER BENCHMARK        ");
    Serial.println("================================");
    Serial.printf("Samples : %u\r\n", static_cast<unsigned>(kSamples));
    Serial.printf("Baseline: %.2f hPa\r\n", kBaseline);
    Serial.println("Step    : +10.00 hPa");
#else
    printf("\n================================\n");
    printf("         FILTER BENCHMARK        \n");
    printf("================================\n");
    printf("Samples : %u\n", static_cast<unsigned>(kSamples));
    printf("Baseline: %.2f hPa\n", kBaseline);
    printf("Step    : +10.00 hPa\n");
#endif
    FilterBenchmarkResult results[kMaxFilters];
    for (size_t i = 0; i < count; ++i) {
      results[i] = run(*filters[i], object_sizes[i]);
#ifdef ARDUINO
      Serial.println();
      Serial.printf("[%s]\r\n", results[i].name);
      Serial.println("  Execution");
      Serial.printf("    RAM      : %u bytes\r\n",
                    static_cast<unsigned>(results[i].object_bytes));
      Serial.printf("    Average  : %.2f us/update\r\n", results[i].average_us);
      Serial.printf("    Peak     : %.2f us/update\r\n", results[i].peak_us);
      Serial.printf("    Heap     : %s\r\n",
                    results[i].allocation_safe ? "none" : "YES");
      Serial.println("  Signal quality");
      Serial.printf("    Sigma    : %.4f hPa\r\n",
                    results[i].standard_deviation);
      Serial.printf("    Spike    : %.3f hPa\r\n",
                    results[i].impulse_deviation);
      Serial.printf("    90%% lag  : %d samples\r\n",
                    results[i].step_lag_iterations);
#else
      printf("\n[%s]\n", results[i].name);
      printf("  Execution\n");
      printf("    RAM      : %u bytes\n",
             static_cast<unsigned>(results[i].object_bytes));
      printf("    Average  : %.2f us/update\n", results[i].average_us);
      printf("    Peak     : %.2f us/update\n", results[i].peak_us);
      printf("    Heap     : %s\n",
             results[i].allocation_safe ? "none" : "YES");
      printf("  Signal quality\n");
      printf("    Sigma    : %.4f hPa\n", results[i].standard_deviation);
      printf("    Spike    : %.3f hPa\n", results[i].impulse_deviation);
      printf("    90%% lag  : %d samples\n", results[i].step_lag_iterations);
#endif
    }
#ifdef ARDUINO
    Serial.println();
    Serial.println("================================");
#else
    printf("\n================================\n\n");
#endif
  }
};
