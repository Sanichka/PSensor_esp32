#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <PressureProcessing.h>

Adafruit_BMP085 bmp_i2c;
Adafruit_BMP280 bmp_spi(5);

bool i2c_online = false;
bool spi_online = false;

KalmanFilter bmpI2C_kalman(0.01f, 0.25f);
KalmanFilter bmpSPI_kalman(0.01f, 0.25f);

#ifdef WOKWI_AUTOTEST
static void runWokwiTest(const String& command);
#endif

void setup() {
  Serial.begin(115200);
  Serial.println("SETUP...");

  Wire.begin(21, 22);
  SPI.begin(18, 19, 23, 5);

  if (bmp_i2c.begin()) {
    i2c_online = true;
    Serial.println("BMP180 I2C online!");
  } else {
    Serial.println("BMP180 I2C not found!");
  }

  if (bmp_spi.begin()) {
    spi_online = true;
    Serial.println("BMP280 SPI online!");
  } else {
    Serial.println("BMP280 SPI not found! -> Fallback data generation enabled.");
  }
#ifdef WOKWI_AUTOTEST
  Serial.println("WOKWI_AUTOTEST_READY");
#endif
}

void loop() {
#ifdef WOKWI_AUTOTEST
  while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    runWokwiTest(command);
  }
#endif

  static uint32_t sample_id = 0;
  sample_id++;

  // Get I2C pressure reading
  float raw_i2c = NAN;
  if (i2c_online) {
    raw_i2c = bmp_i2c.readPressure() / 100.0f; // Pa -> hPa
  }
  SensorData data_i2c = processPressure(bmpI2C_kalman, raw_i2c);

  // Get SPI pressure reading
  float raw_spi = NAN;
  if (spi_online) {
    raw_spi = bmp_spi.readPressure() / 100.0f;
  } else {
    // Data generation for testing only
    float base = (!isnan(raw_i2c)) ? raw_i2c : 1013.25f;
    float noise = ((rand() % 100) - 50) / 100.0f; // noise +-0.5 hPa
    raw_spi = base + 0.2f + noise;

    // spikes every 10th sample
    if (sample_id % 10 == 0) {
      raw_spi += 7.0f;
    }
  }
  SensorData data_spi = processPressure(bmpSPI_kalman, raw_spi);

  // Outptut results
  Serial.printf("\n#%-4u | ", sample_id);
  Serial.printf("I2C: Raw=%6.2f hPa, KF=%6.2f hPa [%-16s] | ", 
                data_i2c.raw_hpa, data_i2c.filtered_hpa, data_i2c.status);
  Serial.printf("SPI: Raw=%6.2f hPa, KF=%6.2f hPa [%-16s]", 
                data_spi.raw_hpa, data_spi.filtered_hpa, data_spi.status);

  // Cross validation between I2C and SPI readings
  if (data_i2c.is_valid && data_spi.is_valid) {
    float delta = fabs(data_i2c.filtered_hpa - data_spi.filtered_hpa);
    if (delta > 3.0f) {
      Serial.printf(" -> [DISCREPANCY WARN: Delta=%.2f hPa!]", delta);
    }
  }
  Serial.println();

  delay(200);
}

#ifdef WOKWI_AUTOTEST
static void printWokwiResult(const char* name, bool passed) {
  Serial.printf("WOKWI_TEST %s %s\n", name, passed ? "PASS" : "FAIL");
}

static bool isNear(float actual, float expected, float tolerance = 0.01f) {
  return fabs(actual - expected) <= tolerance;
}

static void runWokwiTest(const String& command) {
  if (command == "TEST:FIRST") {
    KalmanFilter filter;
    SensorData data = processPressure(filter, 1013.25f);
    printWokwiResult("FIRST_VALID", data.is_valid &&
                                      strcmp(data.status, "OK") == 0 &&
                                      isNear(data.filtered_hpa, 1013.25f));
  } else if (command == "TEST:SMOOTH") {
    KalmanFilter filter;
    processPressure(filter, 1000.0f);
    SensorData data = processPressure(filter, 1002.0f);
    printWokwiResult("SMOOTHING", data.is_valid &&
                                      data.filtered_hpa > 1000.0f &&
                                      data.filtered_hpa < 1002.0f);
  } else if (command == "TEST:DISCONNECT") {
    const float values[] = {NAN, 0.0f, -1.0f};
    bool passed = true;
    for (float value : values) {
      KalmanFilter filter;
      SensorData data = processPressure(filter, value);
      passed = passed && !data.is_valid &&
               strcmp(data.status, "ERR_DISCONNECTED") == 0 &&
               isnan(data.filtered_hpa);
    }
    printWokwiResult("DISCONNECTED_VALUES", passed);
  } else if (command == "TEST:RANGE") {
    const float values[] = {300.0f, 1100.0f};
    bool passed = true;
    for (float value : values) {
      KalmanFilter filter;
      SensorData data = processPressure(filter, value);
      passed = passed && data.is_valid && strcmp(data.status, "OK") == 0;
    }
    printWokwiResult("VALID_BOUNDARIES", passed);
  } else if (command == "TEST:OUT_OF_RANGE") {
    const float values[] = {299.99f, 1100.03f, 1500.0f};
    bool passed = true;
    for (float value : values) {
      KalmanFilter filter;
      SensorData data = processPressure(filter, value);
      passed = passed && !data.is_valid &&
               strcmp(data.status, "ERR_OUT_OF_RANGE") == 0 &&
               isnan(data.filtered_hpa);
    }
    printWokwiResult("OUT_OF_RANGE", passed);
  } else if (command == "TEST:SPIKE") {
    KalmanFilter filter;
    processPressure(filter, 1000.0f);
    SensorData data = processPressure(filter, 1004.01f);
    printWokwiResult("SPIKE_WARNING", data.is_valid &&
                                      strcmp(data.status, "WARN_SPIKE") == 0 &&
                                      data.filtered_hpa > 1000.0f &&
                                      data.filtered_hpa < 1004.01f);
  } else if (command == "TEST:THRESHOLD") {
    KalmanFilter filter;
    processPressure(filter, 1000.0f);
    SensorData data = processPressure(filter, 1004.0f);
    printWokwiResult("SPIKE_THRESHOLD", data.is_valid &&
                                      strcmp(data.status, "OK") == 0);
  } else if (command == "TEST:PRESERVE") {
    KalmanFilter filter;
    processPressure(filter, 1000.0f);
    const float previous = filter.x;
    SensorData data = processPressure(filter, NAN);
    printWokwiResult("INVALID_PRESERVES_STATE", !data.is_valid &&
                                      isNear(filter.x, previous));
  } else if (command == "TEST:RECOVER") {
    KalmanFilter filter;
    processPressure(filter, 1000.0f);
    SensorData disconnected = processPressure(filter, NAN);
    SensorData recovered = processPressure(filter, 1001.0f);
    printWokwiResult("DISCONNECT_RECOVERY", !disconnected.is_valid &&
                                      recovered.is_valid &&
                                      strcmp(recovered.status, "OK") == 0 &&
                                      recovered.filtered_hpa > 1000.0f &&
                                      recovered.filtered_hpa < 1001.0f);
  } else if (command == "TEST:RESET") {
    KalmanFilter filter;
    processPressure(filter, 1000.0f);
    processPressure(filter, 1005.0f);
    filter.reset();
    SensorData data = processPressure(filter, 900.0f);
    printWokwiResult("FILTER_RESET", data.is_valid &&
                                      strcmp(data.status, "OK") == 0 &&
                                      isNear(data.filtered_hpa, 900.0f));
  }
}
#endif