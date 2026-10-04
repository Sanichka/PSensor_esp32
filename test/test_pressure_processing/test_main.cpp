#include <cmath>
#include <PressureProcessing.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_first_valid_reading_initializes_filter() {
  KalmanFilter filter;

  SensorData data = processPressure(filter, 1013.25f);

  TEST_ASSERT_TRUE(data.is_valid);
  TEST_ASSERT_EQUAL_STRING("OK", data.status);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1013.25f, data.filtered_hpa);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1013.25f, filter.x);
}

void test_filter_smooths_subsequent_reading() {
  KalmanFilter filter;
  processPressure(filter, 1000.0f);

  SensorData data = processPressure(filter, 1002.0f);

  TEST_ASSERT_TRUE(data.is_valid);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1001.603f, data.filtered_hpa);
  TEST_ASSERT_TRUE(data.filtered_hpa < 1002.0f);
}

void test_nan_zero_and_negative_values_are_disconnected() {
  const float invalid_values[] = {NAN, 0.0f, -1.0f};

  for (float raw : invalid_values) {
    KalmanFilter filter;
    SensorData data = processPressure(filter, raw);

    TEST_ASSERT_FALSE(data.is_valid);
    TEST_ASSERT_EQUAL_STRING("ERR_DISCONNECTED", data.status);
    TEST_ASSERT_TRUE(std::isnan(data.filtered_hpa));
  }
}

void test_valid_pressure_range_includes_boundaries() {
  const float boundary_values[] = {300.0f, 1100.0f};

  for (float raw : boundary_values) {
    KalmanFilter filter;
    SensorData data = processPressure(filter, raw);

    TEST_ASSERT_TRUE(data.is_valid);
    TEST_ASSERT_EQUAL_STRING("OK", data.status);
  }
}

void test_out_of_range_values_are_rejected() {
  const float invalid_values[] = {299.99f, 1100.03f, 1500.0f};

  for (float raw : invalid_values) {
    KalmanFilter filter;
    SensorData data = processPressure(filter, raw);

    TEST_ASSERT_FALSE(data.is_valid);
    TEST_ASSERT_EQUAL_STRING("ERR_OUT_OF_RANGE", data.status);
    TEST_ASSERT_TRUE(std::isnan(data.filtered_hpa));
  }
}

void test_spike_is_valid_but_reported() {
  KalmanFilter filter;
  processPressure(filter, 1000.0f);

  SensorData data = processPressure(filter, 1004.01f);

  TEST_ASSERT_TRUE(data.is_valid);
  TEST_ASSERT_EQUAL_STRING("WARN_SPIKE", data.status);
  TEST_ASSERT_TRUE(data.filtered_hpa > 1000.0f);
  TEST_ASSERT_TRUE(data.filtered_hpa < 1004.01f);
}

void test_four_hpa_change_is_not_a_spike() {
  KalmanFilter filter;
  processPressure(filter, 1000.0f);

  SensorData data = processPressure(filter, 1004.0f);

  TEST_ASSERT_TRUE(data.is_valid);
  TEST_ASSERT_EQUAL_STRING("OK", data.status);
}

void test_invalid_reading_does_not_change_filter_state() {
  KalmanFilter filter;
  processPressure(filter, 1000.0f);
  const float previous = filter.x;

  SensorData data = processPressure(filter, NAN);

  TEST_ASSERT_FALSE(data.is_valid);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, previous, filter.x);
}

void test_disconnect_during_sequence_does_not_poison_filter() {
  KalmanFilter filter;
  processPressure(filter, 1000.0f);

  SensorData disconnected = processPressure(filter, NAN);
  SensorData recovered = processPressure(filter, 1001.0f);

  TEST_ASSERT_FALSE(disconnected.is_valid);
  TEST_ASSERT_EQUAL_STRING("ERR_DISCONNECTED", disconnected.status);
  TEST_ASSERT_TRUE(recovered.is_valid);
  TEST_ASSERT_EQUAL_STRING("OK", recovered.status);
  TEST_ASSERT_TRUE(recovered.filtered_hpa > 1000.0f);
  TEST_ASSERT_TRUE(recovered.filtered_hpa < 1001.0f);
}

void test_reset_starts_a_new_filter_sequence() {
  KalmanFilter filter;
  processPressure(filter, 1000.0f);
  processPressure(filter, 1005.0f);
  filter.reset();

  SensorData data = processPressure(filter, 900.0f);

  TEST_ASSERT_TRUE(data.is_valid);
  TEST_ASSERT_EQUAL_STRING("OK", data.status);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 900.0f, data.filtered_hpa);
}

void runTests() {
  UNITY_BEGIN();
  RUN_TEST(test_first_valid_reading_initializes_filter);
  RUN_TEST(test_filter_smooths_subsequent_reading);
  RUN_TEST(test_nan_zero_and_negative_values_are_disconnected);
  RUN_TEST(test_valid_pressure_range_includes_boundaries);
  RUN_TEST(test_out_of_range_values_are_rejected);
  RUN_TEST(test_spike_is_valid_but_reported);
  RUN_TEST(test_four_hpa_change_is_not_a_spike);
  RUN_TEST(test_invalid_reading_does_not_change_filter_state);
  RUN_TEST(test_disconnect_during_sequence_does_not_poison_filter);
  RUN_TEST(test_reset_starts_a_new_filter_sequence);
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
