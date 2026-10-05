#include <cmath>
#include <unity.h>

#include <BenchmarkRunner.h>
#include <filters/ComplementaryFusion.h>
#include <filters/EMAFilter.h>
#include <filters/KalmanFilter.h>
#include <filters/MedianFilter.h>
#include <filters/NotchFilter.h>
#include <filters/SMAFilter.h>

void setUp() {}
void tearDown() {}

void test_sma_uses_fixed_window() {
  filters::SMAFilter<5> filter;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, filter.update(2.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, filter.update(3.0f));
}

void test_median_rejects_impulse() {
  filters::MedianFilter<5> filter(5);
  filter.init(1000.0f);
  filter.update(1015.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, filter.update(1000.0f));
}

void test_ema_and_kalman_initialize_on_first_value() {
  filters::EMAFilter ema;
  filters::KalmanFilter kalman;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, ema.update(1000.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, kalman.update(1000.0f));
}

void test_complementary_fusion_accepts_acceleration() {
  filters::ComplementaryFusion filter;
  filter.init(0.0f);
  const float output = filter.update(0.0f, 1.0f, 1.0f);
  TEST_ASSERT_TRUE(output > 0.0f);
}

void test_notch_preserves_dc() {
  filters::NotchFilter filter(100.0f, 10.0f);
  filter.init(1000.0f);
  for (int i = 0; i < 20; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, filter.update(1000.0f));
  }
}

void test_benchmark_reports_sensible_step_lag() {
  filters::EMAFilter filter(0.15f);
  const FilterBenchmarkResult result =
      FilterBenchmark::run(filter, sizeof(filter));
  TEST_ASSERT_TRUE(result.standard_deviation >= 0.0f);
  TEST_ASSERT_TRUE(result.step_lag_iterations > 0);
  TEST_ASSERT_TRUE(result.allocation_safe);
}

void runTests() {
  UNITY_BEGIN();
  RUN_TEST(test_sma_uses_fixed_window);
  RUN_TEST(test_median_rejects_impulse);
  RUN_TEST(test_ema_and_kalman_initialize_on_first_value);
  RUN_TEST(test_complementary_fusion_accepts_acceleration);
  RUN_TEST(test_notch_preserves_dc);
  RUN_TEST(test_benchmark_reports_sensible_step_lag);
}

#ifdef ARDUINO
void setup() {
  runTests();
  UNITY_END();
}
void loop() {}
#else
int main() {
  runTests();
  return UNITY_END();
}
#endif
