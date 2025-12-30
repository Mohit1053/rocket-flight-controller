/*************************************************************
 *  CanSat Flight Controller + GPS Logger (CSV Output)
 *  ----------------------------------------------------------
 *  - Reads altitude from BMP388 via SPI
 *  - Detects launch, apogee, descent using filtered altitude
 *  - Controls two Feetech smart servos for CanSat + chute
 *  - Parses GPS NMEA ($GNRMC, $GNGGA)
 *  - Outputs combined telemetry as CSV
 *************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SCServo.h>
#include <HardwareSerial.h>
#include <LoRa.h>
#include <TinyGPS++.h>

// --- TEST MODE (1 = indoor simulation, 0 = real flight) ---
#define TEST_MODE 1

// --- BMP388 SPI pins ---
#define BMP_CS   7
#define BMP_SCK  13
#define BMP_MISO 12
#define BMP_MOSI 11

Adafruit_BMP3XX bmp;

// --- LoRa SPI Configuration (SPI3: PC12/PC11/PC10) ---
SPIClass LoRaSPI(PC12, PC11, PC10); // MOSI, MISO, SCK
#define LORA_SS   PD15
#define LORA_RST  PD14
#define LORA_DIO0 PE9

// --- Servo Bus (Half-Duplex UART) ---
SMS_STS servoBus;
HardwareSerial servoSerial(PA0, PA1);  // TX=PA1, RX=PA0

struct ServoAction {
  uint8_t id;
  uint16_t safePos;
  uint16_t firePos;
  uint16_t moveTimeMs;
  uint16_t holdTimeMs;
  uint16_t speed;
  bool hasFired;
  bool isHolding;
  unsigned long holdStartTime;
};

// Define CanSat ejection servo (ID 1)
ServoAction cansatServo = {
  1, 2048, 2600, 1500, 400, 250, false, false, 0
};

// Define Parachute deployment servo (ID 2)
ServoAction chuteServo = {
  2, 2048, 3000, 1500, 400, 250, false, false, 0
};

/* -----------------------------------------------------------
   FLIGHT STATE MACHINE PARAMETERS
----------------------------------------------------------- */
#if TEST_MODE
const float LAUNCH_ALT_M = 2.0;
const uint8_t LAUNCH_CONFIRM = 3;
const float APOGEE_DROP_M = 0.7;
const float VZ_MIN_MPS = 0.05;
const uint8_t APOGEE_CONFIRM = 3;
const float CHUTE_DEPLOY_ALT_M = 5.0;
const uint8_t DESCENT_CONFIRM = 2;
const unsigned long FAILSAFE_DELAY_MS = 15000UL;
float alphaAlt = 0.25;
float maxJumpAlt = 0.5;
#else
const float LAUNCH_ALT_M = 30.0;
const uint8_t LAUNCH_CONFIRM = 5;
const float APOGEE_DROP_M = 4.0;
const float VZ_MIN_MPS = 0.3;
const uint8_t APOGEE_CONFIRM = 5;
const float CHUTE_DEPLOY_ALT_M = 500.0;
const uint8_t DESCENT_CONFIRM = 3;
const unsigned long FAILSAFE_DELAY_MS = 100000UL;
float alphaAlt = 0.2;
float maxJumpAlt = 1.5;
#endif

enum FlightState { IDLE, ASCENT, APOGEE, DESCENT, CHUTE_DEPLOYED };
FlightState flightState = IDLE;

/* -----------------------------------------------------------
   ALTITUDE & KINEMATICS VARIABLES
----------------------------------------------------------- */
float altitudeFiltered = 0;
float altitudeZeroOffset = 0;
float launchPressure_hPa = 1013.25;
bool calibrated = false;
float verticalSpeed = 0;
float prevAltitude = 0;
const float VZ_EMA_BETA = 0.2;
unsigned long lastUpdateMs = 0;
float peakAltitude = 0;
uint8_t launchCount = 0;
uint8_t apogeeCount = 0;
uint8_t descentCount = 0;
bool failsafeArmed = false;
unsigned long failsafeStartMs = 0;

float filterAltitude(float rawAlt) {
  float diff = rawAlt - altitudeFiltered;
  if (fabs(diff) > maxJumpAlt)
    rawAlt = altitudeFiltered + (diff > 0 ? maxJumpAlt : -maxJumpAlt);
  altitudeFiltered = alphaAlt * rawAlt + (1 - alphaAlt) * altitudeFiltered;
  return altitudeFiltered;
}

void autoCalibrate() {
  Serial.println(F("[CAL] Calibrating BMP388... Keep still for 3s."));
  const int samples = 100;
  float pressureSum = 0;
  int valid = 0;
  for (int i = 0; i < samples; i++) {
    if (bmp.performReading()) {
      pressureSum += bmp.pressure;
      valid++;
    }
    delay(30);
  }
  if (valid > 10) {
    launchPressure_hPa = (pressureSum / valid) / 100.0;
    Serial.print(F("[CAL] Baseline Pressure = "));
    Serial.print(launchPressure_hPa, 2);
    Serial.println(F(" hPa"));
  } else {
    Serial.println(F("[CAL] Warning: Low sample count; using default baseline."));
  }
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
  Serial.print(F("[CAL] Ground offset = "));
  Serial.println(altitudeZeroOffset, 2);
}

void moveServoToFire(ServoAction &servo, const __FlashStringHelper* name) {
  if (servo.hasFired) return;
  servo.hasFired = true;
  servo.isHolding = true;
  servo.holdStartTime = millis();
  servoBus.WritePosEx(servo.id, servo.firePos, servo.moveTimeMs, servo.speed);
}

void updateServoReturn(ServoAction &servo) {
  if (servo.isHolding && (millis() - servo.holdStartTime >= servo.holdTimeMs)) {
    servo.isHolding = false;
    servoBus.WritePosEx(servo.id, servo.safePos, servo.moveTimeMs, servo.speed);
  }
}

/* -----------------------------------------------------------
   GPS MODULE (USART2: PA2=TX, PA3=RX) - TinyGPS++
----------------------------------------------------------- */
HardwareSerial gpsSerial(USART2);
TinyGPSPlus gps;

// GPS data variables
double lastLat = 0, lastLon = 0;
double lastSpeed = 0;
float lastGPSAlt = 0;
int lastHour = 0, lastMinute = 0, lastSecond = 0;

void convertUTCtoIST(int &hour, int &minute, int &second) {
  hour += 5; minute += 30;
  if (minute >= 60) { minute -= 60; hour += 1; }
  if (hour >= 24) hour -= 24;
}

/* -----------------------------------------------------------
   SETUP
----------------------------------------------------------- */
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);  // TinyGPS++ uses 9600 baud
  while (!Serial);

  Serial.println(F("Hour,Minute,Second,Latitude,Longitude,Speed(km/h),GPS_Alt(m),"
                   "Pressure(hPa),Temp(C),Alt(m),VZ(m/s),PeakAlt,State"));

  // LoRa setup
  LoRaSPI.begin();
  LoRa.setSPI(LoRaSPI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(433E6)) {
    Serial.println(F("[BOOT] ERROR: LoRa initialization failed!"));
    while (1) delay(10);
  }
  Serial.println(F("[BOOT] LoRa initialized at 433 MHz"));

  // Servo setup
  servoSerial.begin(1000000);
  servoBus.pSerial = &servoSerial;
  delay(100);
  for (uint8_t id : {cansatServo.id, chuteServo.id}) {
    servoBus.EnableTorque(id, 1);
    delay(20);
  }
  servoBus.WritePosEx(cansatServo.id, cansatServo.safePos, 1500, 250);
  servoBus.WritePosEx(chuteServo.id, chuteServo.safePos, 1500, 250);

  // BMP setup
  if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
    Serial.println(F("[BOOT] ERROR: BMP388 not detected!"));
    while (1) delay(10);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_31);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  autoCalibrate();
  lastUpdateMs = millis();
}

/* -----------------------------------------------------------
   LOOP
----------------------------------------------------------- */
void loop() {
  // --- GPS Reading with TinyGPS++ ---
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // Update GPS data if valid
  if (gps.location.isUpdated()) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
  }
  if (gps.altitude.isUpdated()) {
    lastGPSAlt = gps.altitude.meters();
  }
  if (gps.speed.isUpdated()) {
    lastSpeed = gps.speed.kmph();
  }
  if (gps.time.isUpdated()) {
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();
    convertUTCtoIST(hour, minute, second);
    lastHour = hour;
    lastMinute = minute;
    lastSecond = second;
  }

  // --- BMP388 Reading ---
  if (!bmp.performReading()) return;
  float rawAlt = bmp.readAltitude(launchPressure_hPa) - altitudeZeroOffset;
  float filteredAlt = filterAltitude(rawAlt);
  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0;
  if (dt < 0.001) dt = 0.001;
  float vz_raw = (filteredAlt - prevAltitude) / dt;
  verticalSpeed = (1 - VZ_EMA_BETA) * verticalSpeed + VZ_EMA_BETA * vz_raw;
  prevAltitude = filteredAlt;
  lastUpdateMs = now;

  // --- Flight State Machine ---
  switch (flightState) {
    case IDLE:
      if (filteredAlt > LAUNCH_ALT_M) {
        launchCount++;
        if (launchCount >= LAUNCH_CONFIRM) {
          flightState = ASCENT;
          peakAltitude = filteredAlt;
          failsafeArmed = false;
        }
      } else launchCount = 0;
      break;

    case ASCENT:
      if (filteredAlt > peakAltitude) peakAltitude = filteredAlt;
      if (verticalSpeed < -VZ_MIN_MPS) apogeeCount++;
      else apogeeCount = 0;
      if (apogeeCount >= APOGEE_CONFIRM && (peakAltitude - filteredAlt) > APOGEE_DROP_M) {
        moveServoToFire(cansatServo, F("CanSat"));
        flightState = APOGEE;
        failsafeArmed = true;
        failsafeStartMs = millis();
      }
      break;

    case APOGEE:
      if (verticalSpeed < -VZ_MIN_MPS) {
        descentCount++;
        if (descentCount >= DESCENT_CONFIRM) flightState = DESCENT;
      } else descentCount = 0;
      break;

    case DESCENT:
      if (verticalSpeed < -VZ_MIN_MPS && filteredAlt <= CHUTE_DEPLOY_ALT_M && !chuteServo.hasFired) {
        moveServoToFire(chuteServo, F("Parachute"));
        flightState = CHUTE_DEPLOYED;
      }
      if (failsafeArmed && !chuteServo.hasFired && (millis() - failsafeStartMs) >= FAILSAFE_DELAY_MS) {
        moveServoToFire(chuteServo, F("Parachute"));
        flightState = CHUTE_DEPLOYED;
      }
      break;
    case CHUTE_DEPLOYED:
      break;
  }

  updateServoReturn(cansatServo);
  updateServoReturn(chuteServo);

  // --- LoRa Telemetry Transmission ---
  String telemetry = String(lastHour) + "," + String(lastMinute) + "," + String(lastSecond) + "," +
                     String(lastLat, 6) + "," + String(lastLon, 6) + "," + String(lastSpeed, 2) + "," +
                     String(lastGPSAlt, 2) + "," + String(bmp.pressure / 100.0, 2) + "," +
                     String(bmp.temperature, 2) + "," + String(filteredAlt, 2) + "," +
                     String(verticalSpeed, 2) + "," + String(peakAltitude, 2) + "," +
                     ((flightState == IDLE) ? "IDLE" :
                      (flightState == ASCENT) ? "ASCENT" :
                      (flightState == APOGEE) ? "APOGEE" :
                      (flightState == DESCENT) ? "DESCENT" : "CHUTE_DEPLOYED");
  
  LoRa.beginPacket();
  LoRa.print(telemetry);
  LoRa.endPacket();

  // --- CSV Telemetry Output ---
  Serial.print(lastHour); Serial.print(",");
  Serial.print(lastMinute); Serial.print(",");
  Serial.print(lastSecond); Serial.print(",");
  Serial.print(lastLat, 6); Serial.print(",");
  Serial.print(lastLon, 6); Serial.print(",");
  Serial.print(lastSpeed, 2); Serial.print(",");
  Serial.print(lastGPSAlt, 2); Serial.print(",");
  Serial.print(bmp.pressure / 100.0, 2); Serial.print(",");
  Serial.print(bmp.temperature, 2); Serial.print(",");
  Serial.print(filteredAlt, 2); Serial.print(",");
  Serial.print(verticalSpeed, 2); Serial.print(",");
  Serial.print(peakAltitude, 2); Serial.print(",");
  Serial.println(
    (flightState == IDLE) ? "IDLE" :
    (flightState == ASCENT) ? "ASCENT" :
    (flightState == APOGEE) ? "APOGEE" :
    (flightState == DESCENT) ? "DESCENT" :
    "CHUTE_DEPLOYED"
  );

  delay(100);
}
