#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

// --- SPI pin definitions for Nucleo (Arduino pin mapping) ---
#define BMP_CS   7
#define BMP_SCK  13
#define BMP_MISO 12
#define BMP_MOSI 11

Adafruit_BMP3XX bmp;

// --- Filtering parameters (tuned for faster response) ---
float alphaAlt = 0.2;       // EMA smoothing factor (higher = more responsive)
float maxJumpAlt = 1.5;      // meters per update limit
float altitudeFiltered = 0;
float altitudeZeroOffset = 0;
float launchPressure_hPa = 1013.25; // temporary default, recalibrated automatically
bool calibrated = false;

// --- Helper: spike-resistant low-pass filter ---
float filterAltitude(float raw) {
  float diff = raw - altitudeFiltered;
  if (fabs(diff) > maxJumpAlt)
    raw = altitudeFiltered + (diff > 0 ? maxJumpAlt : -maxJumpAlt);
  altitudeFiltered = alphaAlt * raw + (1 - alphaAlt) * altitudeFiltered;
  return altitudeFiltered;
}

// --- Auto-calibration routine ---
void autoCalibrate() {
  Serial.println(F("Calibrating BMP388... Keep still for 3s."));
  const int samples = 100;
  float sum = 0, valid = 0;

  for (int i = 0; i < samples; i++) {
    if (bmp.performReading()) {
      sum += bmp.pressure;
      valid++;
    }
    delay(30);
  }

  if (valid > 10) {
    launchPressure_hPa = (sum / valid) / 100.0;
    Serial.print(F("Auto baseline pressure = "));
    Serial.print(launchPressure_hPa, 2);
    Serial.println(F(" hPa"));
  } else {
    Serial.println(F("Calibration failed, using last known baseline."));
  }

  // Zero the ground altitude
  float altSum = 0;
  int count = 0;
  for (int i = 0; i < 250; i++) {
    if (bmp.performReading()) {
      altSum += bmp.readAltitude(launchPressure_hPa);
      count++;
    }
    delay(20);
  }
  altitudeZeroOffset = (count > 0) ? altSum / count : 0;
  calibrated = true;
  Serial.println("CALIBRATION COMPLETE");

  Serial.print(F("Ground altitude offset = "));
  Serial.println(altitudeZeroOffset, 2);
  Serial.println(F("Calibration complete.\n"));
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("BMP388 Altitude System Initializing..."));

  if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
    Serial.println(F("BMP388 not found! Check wiring."));
    while (1) delay(10);
  }

  // Optimized sensor configuration (fast + stable)
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_31);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  autoCalibrate();
}

// --- Main Loop ---
void loop() {
  if (!bmp.performReading()) return;

  // Compute altitude relative to baseline
  float rawAlt = bmp.readAltitude(launchPressure_hPa) - altitudeZeroOffset;

  // Apply dynamic filter
  float filteredAlt = filterAltitude(rawAlt);

  // Optional: Adaptive alpha (faster when large change)
  float delta = fabs(rawAlt - altitudeFiltered);
  alphaAlt = (delta > 0.5) ? 0.3 : 0.2;

  // Display clean telemetry
  Serial.print(calibrated);
  Serial.print("Pressure(hPa): ");
  Serial.print(bmp.pressure / 100.0, 2);
  Serial.print(" | Temp(C): ");
  Serial.print(bmp.temperature, 2);
  Serial.print(" | Alt(m): ");
  Serial.println(filteredAlt, 2);

  delay(100); // ~10 Hz output for faster response
}
