#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>

Adafruit_BMP085 bmp_i2c;
Adafruit_BMP280 bmp_spi(5);

bool i2c_online = false;
bool spi_online = false;

struct KalmanFilter {
  float x; // Pressure Value(current state estimate)
  float p; // Error Covariance (Estimate Uncertainty)
  float q; // Noise Covariance (Process Noise Covariance)
  float r; // Measurement Noise Covariance (Sensor Noise Covariance)
  float k; // Kalman Gain

  // Default constuructor with default parameters for process noise, sensor noise, and initial estimate error
  KalmanFilter(float process_noise = 0.01f, float sensor_noise = 0.25f, float init_est_error = 1.0f) {
    x = NAN;
    q = process_noise;
    r = sensor_noise;
    p = init_est_error;
    k = 0.0f;
  }

  float update(float measurement) {
    // init if it's first read
    if (isnan(x)) {
      x = measurement;
      return x;
    }
    // 1. Prediction update
    p = p + q;

    // 2. Kalman Gain
    k = p / (p + r);

    // 3. Update with new measurement
    x = x + k * (measurement - x);

    // 4. Update the error covariance
    p = (1.0f - k) * p;

    return x;
  }

  void reset() {
    x = NAN;
    p = 1.0f;
  }
};

KalmanFilter bmpI2C_kalman(0.01f, 0.25f);
KalmanFilter bmpSPI_kalman(0.01f, 0.25f);

struct SensorData {
  float raw_hpa;
  float filtered_hpa;
  bool is_valid;
  const char* status;
};

SensorData processPressure(KalmanFilter &kf, float raw) {
  SensorData data = {raw, NAN, false, "ERR_UNKNOWN"};

  if (isnan(raw) || raw <= 0.0f) {
    data.status = "ERR_DISCONNECTED";
    return data;
  }

  // 1. Sensor range (300 ... 1100 hPa)
  if (raw < 300.0f || raw > 1100.1f) {
    data.status = "ERR_OUT_OF_RANGE";
    return data;
  }

  // 2. Random spikes (> 4 hPa from current filtered value)
  if (!isnan(kf.x) && fabs(raw - kf.x) > 4.0f) {
    data.status = "WARN_SPIKE";
  } else {
    data.status = "OK";
  }

  data.filtered_hpa = kf.update(raw);
  data.is_valid = true;
  return data;
}

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
}

void loop() {
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