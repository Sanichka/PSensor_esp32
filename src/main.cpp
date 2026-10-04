#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

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

KalmanFilter bmp_kalman(0.01f, 0.25f);

// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(115200);
  Serial.println("SETUP...");

  Wire.begin(21, 22);

  if (!bmp.begin()) {
    Serial.println("BMP180 not found!");
    while (1) {
      delay(500);
    }
  }

  Serial.println("BMP180 found!");
}

void loop() {
  int32_t pressure = bmp.readPressure();
  float raw_hpa = pressure / 100.0f; // hectopascal measurement

  float filtered_hpa = bmp_kalman.update(raw_hpa);
  Serial.printf("Raw: %.2f hPa | Filtered KF: %.2f hPa\r\n", raw_hpa, filtered_hpa);
  delay(200);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}