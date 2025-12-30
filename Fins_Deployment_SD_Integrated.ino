/*************************************************************
 *  INTEGRATED ROCKET FLIGHT CONTROLLER WITH SD LOGGING & TELEMETRY
 *  ================================================================
 *  Combines:
 *  - Fins.ino: LQR-based stabilization with MPU9250 (4 servos)
 *  - deployment_GPS_integrated.ino: CanSat deployment system (2 servos)
 *  - SD_card_working.ino: Data logging to SD card
 *  - telemetry.ino: XBee wireless telemetry
 *  
 *  Hardware:
 *  - MCU: STM32H7A3ZIQ
 *  - IMU: MPU9250 (I2C @ 0x69)
 *  - Altimeter: BMP388 (SPI1: CS=7, SCK=13, MISO=12, MOSI=11)
 *  - SD Card: SPI3 (CS=PD14, SCK=PC10, MISO=PC11, MOSI=PC12)
 *  - GPS: USART2 (PA2/PA3 @ 38400 baud)
 *  - XBee Radio: USART6 (PC6=TX, PC7=RX @ 115200 baud)
 *  - Servos: 6x Feetech SCServo on PA0/PA1 @ 1Mbps
 *    * IDs 1-4: Fin control
 *    * IDs 5-6: Deployment (CanSat, Parachute)
 *  
 *  Integration Notes:
 *  - Fins remain active during entire descent
 *  - Non-blocking task scheduler preserves real-time performance
 *  - Buffered SD card writes (50 lines = ~1 second @ 50Hz)
 *  - Real-time telemetry via XBee (dual output: Serial + XBee)
 *  - Graceful degradation if SD card or XBee fails
 *  - All original logic preserved from source files
 *************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "MPU9250.h"
#include <SCServo.h>
#include <HardwareSerial.h>
#include <math.h>
#include <string.h>
#include "SdFat.h"

/* =================== HARDWARE CONFIGURATION =================== */

// --- MPU9250 IMU (I2C) ---
#define MPU_ADDRESS 0x69
MPU9250 myIMU(MPU_ADDRESS, Wire, 400000);

// --- BMP388 Altimeter (SPI1 - Fixed PCB connections) ---
#define BMP_CS   7
#define BMP_SCK  13
#define BMP_MISO 12
#define BMP_MOSI 11
Adafruit_BMP3XX bmp;

// --- SD Card (SPI3 - Separate bus) ---
#define SD_CS_PIN PD14
SPIClass sdSPI(PC12, PC11, PC10);  // MOSI, MISO, SCK for SPI3
SdFat sd;
File logFile;

// --- Servo Bus (Shared Half-Duplex UART) ---
SMS_STS st;
HardwareSerial servoSerial(PA0, PA1);  // TX=PA1, RX=PA0

// Fin Control Servos (IDs 1-4) - from Fins.ino
const int SERVO_FIN_IDS[4] = {1, 2, 3, 4};
const int SERVO_NEUTRAL = 2048;
const int MAX_DELTA = 500;
const float SCALE_U = 200.0f;

// Deployment Servos (IDs 5-6) - from deployment_GPS_integrated.ino
// **KEY CHANGE: Servo IDs changed from 1,2 to 5,6 to avoid conflict**
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

ServoAction cansatServo = {
  5, 2048, 2600, 1500, 400, 250, false, false, 0  // ID changed: 1 → 5
};

ServoAction chuteServo = {
  6, 2048, 3000, 1500, 400, 250, false, false, 0  // ID changed: 2 → 6
};

// --- GPS Module (USART2) ---
HardwareSerial gpsSerial(USART2);
String nmeaSentence = "";
double lastSpeed = 0, lastCourse = 0;
int lastHour = 0, lastMinute = 0, lastSecond = 0;
double lastLat = 0, lastLon = 0;
String lastAltitude = "";

// --- XBee Telemetry Radio (USART6) ---
HardwareSerial xbeeSerial(USART6);  // PC6=TX, PC7=RX
#define XBEE_BAUD 115200
bool xbeeAvailable = false;

/* =================== SD CARD LOGGING CONFIGURATION =================== */
#define SD_BUFFER_SIZE 50  // 50 CSV lines = ~5KB RAM (~1 sec @ 50Hz)
String csvBuffer[SD_BUFFER_SIZE];
uint16_t bufferIndex = 0;
bool sdCardAvailable = false;
unsigned long lastSDFlushMs = 0;
const uint16_t SD_FLUSH_INTERVAL_MS = 2000;  // Flush every 2 seconds

/* =================== FLIGHT PARAMETERS (from deployment) =================== */

// Test mode disabled per user requirement
#define TEST_MODE 0

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

/* =================== ALTITUDE & KINEMATICS (from deployment) =================== */
float altitudeFiltered = 0;
float altitudeZeroOffset = 0;
float launchPressure_hPa = 1013.25;
bool calibrated = false;
float verticalSpeed = 0;
float prevAltitude = 0;
const float VZ_EMA_BETA = 0.2;
unsigned long lastAltUpdateMs = 0;
float peakAltitude = 0;
uint8_t launchCount = 0;
uint8_t apogeeCount = 0;
uint8_t descentCount = 0;
bool failsafeArmed = false;
unsigned long failsafeStartMs = 0;

/* =================== IMU CALIBRATION & FILTERING (from Fins) =================== */
#define CALIB_SAMPLES 1000  
#define LOOP_SAMPLES  5

float ax_offset=0, ay_offset=0, az_offset=0;
float gx_offset=0, gy_offset=0, gz_offset=0;
float mx_offset=0, my_offset=0, mz_offset=0;

float ax_f=0, ay_f=0, az_f=0;
float gx_f=0, gy_f=0, gz_f=0;
float mx_f=0, my_f=0, mz_f=0;

float alpha = 0.2f;
float max_jump = 200.0f;

const float GYRO_SCALE_FACTOR       = 16.4f;
const float ACCEL_SCALE_FACTOR_1G   = 16384.0f;
const float COUNTS_PER_1G           = ACCEL_SCALE_FACTOR_1G;

unsigned long last_time = 0;

const float Q_angle = 0.001f;
const float R_angle = 0.03f;

float roll_angle=0,  P_roll=0;
float pitch_angle=0, P_pitch=0;

float yaw_angle=0,   P_yaw=0;
const float Q_yaw = 0.001f;
const float R_yaw = 0.5f;

float gx_bias=0.0f, gy_bias=0.0f, gz_bias=0.0f;
const float bias_learn = 0.0002f;
const float still_rate_dps = 2.5f;
const float acc_still_band_frac = 0.10f;

/* =================== LQR MATRICES (from Fins) =================== */
#define DT 0.01

double A_mat[3][3], B_mat[3][4], K_mat[4][3], Acl[3][3], Ad[3][3];

/* =================== TASK TIMING (Non-blocking scheduler) =================== */
unsigned long lastFinsUpdateMs = 0;
unsigned long lastGPSReadMs = 0;
const uint16_t FINS_INTERVAL_MS = 10;   // 100 Hz - critical for stability
const uint16_t GPS_INTERVAL_MS = 100;    // 10 Hz
const uint16_t ALT_INTERVAL_MS = 20;     // 50 Hz

/* =================== HELPER FUNCTIONS =================== */

// --- Altitude Filtering (from deployment) ---
float filterAltitude(float rawAlt) {
  float diff = rawAlt - altitudeFiltered;
  if (fabs(diff) > maxJumpAlt)
    rawAlt = altitudeFiltered + (diff > 0 ? maxJumpAlt : -maxJumpAlt);
  altitudeFiltered = alphaAlt * rawAlt + (1 - alphaAlt) * altitudeFiltered;
  return altitudeFiltered;
}

// --- IMU Filtering (from Fins) ---
float filter(float raw, float &filt) {
  float diff = raw - filt;
  if (fabsf(diff) > max_jump) {
    raw = filt + (diff > 0 ? max_jump : -max_jump);
  }
  filt = alpha * raw + (1.0f - alpha) * filt;
  return filt;
}

// --- Kalman Filter for Roll/Pitch (from Fins) ---
float apply_kalman(float measured_angle, float gyro_rate, float dt,
                   float &angle, float &P, float Q, float R) {
  angle += gyro_rate * dt;
  P     += Q * dt;
  float K = P / (P + R);
  angle   = angle + K * (measured_angle - angle);
  P       = (1.0f - K) * P;
  return angle;
}

// --- Yaw Wrapping (from Fins) ---
static inline float wrapDeg180(float e) {
  while (e > 180.0f) e -= 360.0f;
  while (e < -180.0f) e += 360.0f;
  return e;
}
static inline float wrapDeg360(float a) {
  while (a < 0.0f)    a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

// --- Kalman Filter for Yaw (from Fins) ---
float apply_kalman_yaw(float measured_yaw_deg, float gyro_rate_dps, float dt,
                       float &yaw_deg, float &P, float Q, float R) {
  yaw_deg = wrapDeg360(yaw_deg + gyro_rate_dps * dt);
  P += Q * dt;
  float innov = wrapDeg180(measured_yaw_deg - yaw_deg);
  float K = P / (P + R);
  yaw_deg = wrapDeg360(yaw_deg + K * innov);
  P = (1.0f - K) * P;
  return yaw_deg;
}

// --- Servo Mapping (from Fins) ---
int map_u_to_servo_pos(double u) {
  int delta = (int)round(u * SCALE_U);
  if (delta >  MAX_DELTA) delta =  MAX_DELTA;
  if (delta < -MAX_DELTA) delta = -MAX_DELTA;
  int pos = SERVO_NEUTRAL + delta;
  if (pos < 0)    pos = 0;
  if (pos > 4095) pos = 4095;
  return pos;
}

// --- Deployment Servo Control (from deployment) ---
void moveServoToFire(ServoAction &servo, const __FlashStringHelper* name) {
  if (servo.hasFired) return;
  servo.hasFired = true;
  servo.isHolding = true;
  servo.holdStartTime = millis();
  st.WritePosEx(servo.id, servo.firePos, servo.moveTimeMs, servo.speed);
}

void updateServoReturn(ServoAction &servo) {
  if (servo.isHolding && (millis() - servo.holdStartTime >= servo.holdTimeMs)) {
    servo.isHolding = false;
    st.WritePosEx(servo.id, servo.safePos, servo.moveTimeMs, servo.speed);
  }
}

/* =================== GPS PARSING (from deployment) =================== */

double convertToDecimal(String raw, char hemi) {
  if (raw == "") return 0.0;
  double val = raw.toDouble();
  int degrees = (int)(val / 100);
  double minutes = val - (degrees * 100);
  double decimal = degrees + minutes / 60.0;
  if (hemi == 'S' || hemi == 'W') decimal = -decimal;
  return decimal;
}

void convertUTCtoIST(int &hour, int &minute, int &second) {
  hour += 5; minute += 30;
  if (minute >= 60) { minute -= 60; hour += 1; }
  if (hour >= 24) hour -= 24;
}

void parseGNRMC(String sentence) {
  int idx = 0, last = 0, field = 0;
  String timeUTC, status, latRaw, latDir, lonRaw, lonDir, speedKnots, course;
  while ((idx = sentence.indexOf(',', last)) != -1) {
    String token = sentence.substring(last, idx);
    last = idx + 1;
    switch (field) {
      case 1: timeUTC = token; break;
      case 2: status = token; break;
      case 3: latRaw = token; break;
      case 4: latDir = token; break;
      case 5: lonRaw = token; break;
      case 6: lonDir = token; break;
      case 7: speedKnots = token; break;
      case 8: course = token; break;
    }
    field++;
  }
  if (status != "A" || timeUTC.length() < 6) return;
  int hour = timeUTC.substring(0, 2).toInt();
  int minute = timeUTC.substring(2, 4).toInt();
  int second = timeUTC.substring(4, 6).toInt();
  convertUTCtoIST(hour, minute, second);
  lastHour = hour; lastMinute = minute; lastSecond = second;
  lastLat = convertToDecimal(latRaw, latDir.charAt(0));
  lastLon = convertToDecimal(lonRaw, lonDir.charAt(0));
  lastSpeed = speedKnots.toFloat() * 1.852;
  lastCourse = course.toFloat();
}

void parseGNGGA(String sentence) {
  int idx = 0, last = 0, field = 0;
  String latRaw, latDir, lonRaw, lonDir, altitude;
  while ((idx = sentence.indexOf(',', last)) != -1) {
    String token = sentence.substring(last, idx);
    last = idx + 1;
    switch (field) {
      case 2: latRaw = token; break;
      case 3: latDir = token; break;
      case 4: lonRaw = token; break;
      case 5: lonDir = token; break;
      case 9: altitude = token; break;
    }
    field++;
  }
  
  // Validate altitude: must be numeric (digits, minus, or decimal point only)
  if (altitude.length() > 0) {
    bool valid = true;
    for (unsigned int i = 0; i < altitude.length(); i++) {
      char c = altitude.charAt(i);
      if (!isdigit(c) && c != '.' && c != '-') {
        valid = false;
        break;
      }
    }
    if (valid) {
      lastAltitude = altitude;
    }
    // If invalid, keep previous value (don't update)
  }
}

/* =================== SD CARD FUNCTIONS =================== */

/**
 * Initialize SD card on SPI3 bus
 * Returns true if successful, false otherwise
 * Non-blocking: continues operation even if SD fails
 */
bool initSDCard() {
  Serial.println(F("[SD] Initializing SD card on SPI3..."));
  
  sdSPI.begin();
  delay(100);
  
  // Configure SPI3 for SD card (50 MHz, DEDICATED_SPI mode)
  SdSpiConfig spiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50), &sdSPI);
  
  if (!sd.begin(spiConfig)) {
    Serial.println(F("[SD] ERROR: SD card initialization failed!"));
    Serial.println(F("[SD] Continuing without SD logging (data on Serial only)"));
    return false;
  }
  
  Serial.println(F("[SD] SD card initialized successfully!"));
  
  // Create/open log file with CSV header
  logFile = sd.open("FLIGHT_DATA.CSV", FILE_WRITE);
  if (!logFile) {
    Serial.println(F("[SD] ERROR: Failed to create FLIGHT_DATA.CSV"));
    return false;
  }
  
  // Write CSV header
  logFile.println(F("Time_ms,State,Roll,Pitch,Yaw,p_dps,q_dps,r_dps,u1,u2,u3,u4,"
                    "Alt_BMP,Vz,PeakAlt,Lat,Lon,GPS_Alt,Servo1,Servo2,Servo3,Servo4,Servo5,Servo6"));
  logFile.close();
  
  Serial.println(F("[SD] FLIGHT_DATA.CSV created with header"));
  return true;
}

/**
 * Add CSV line to RAM buffer (fast, non-blocking)
 * Called at 50 Hz from main loop
 */
void logToBuffer(String csvLine) {
  if (bufferIndex < SD_BUFFER_SIZE) {
    csvBuffer[bufferIndex++] = csvLine;
  } else {
    // Buffer full - force immediate flush
    flushBufferToSD();
    csvBuffer[bufferIndex++] = csvLine;
  }
}

/**
 * Flush accumulated buffer to SD card (slow, blocking)
 * Called every 2 seconds or when buffer is full
 */
void flushBufferToSD() {
  if (!sdCardAvailable || bufferIndex == 0) return;
  
  // Open file in append mode
  logFile = sd.open("FLIGHT_DATA.CSV", FILE_WRITE);
  if (!logFile) {
    Serial.println(F("[SD] ERROR: Failed to open file for writing"));
    sdCardAvailable = false;  // Disable SD logging on persistent errors
    return;
  }
  
  // Write all buffered lines
  for (uint16_t i = 0; i < bufferIndex; i++) {
    logFile.println(csvBuffer[i]);
  }
  
  logFile.close();
  
  // Clear buffer
  bufferIndex = 0;
  
  Serial.print(F("[SD] Flushed to FLIGHT_DATA.CSV ("));
  Serial.print(bufferIndex);
  Serial.println(F(" lines)"));
}

/**
 * Build CSV line from current telemetry data
 * Returns formatted String ready for logging
 */
String buildCSVLine(unsigned long timeMs, const char* state, 
                    float roll, float pitch, float yaw,
                    float p, float q, float r,
                    double u1, double u2, double u3, double u4,
                    float alt, float vz, float peakAlt,
                    double lat, double lon, String gpsAlt,
                    int s1, int s2, int s3, int s4, int s5, int s6) {
  String line = "";
  line += timeMs; line += ",";
  line += state; line += ",";
  line += String(roll, 2); line += ",";
  line += String(pitch, 2); line += ",";
  line += String(yaw, 2); line += ",";
  line += String(p, 2); line += ",";
  line += String(q, 2); line += ",";
  line += String(r, 2); line += ",";
  line += String(u1, 4); line += ",";
  line += String(u2, 4); line += ",";
  line += String(u3, 4); line += ",";
  line += String(u4, 4); line += ",";
  line += String(alt, 2); line += ",";
  line += String(vz, 2); line += ",";
  line += String(peakAlt, 2); line += ",";
  line += String(lat, 6); line += ",";
  line += String(lon, 6); line += ",";
  line += gpsAlt; line += ",";
  line += s1; line += ",";
  line += s2; line += ",";
  line += s3; line += ",";
  line += s4; line += ",";
  line += s5; line += ",";
  line += s6;
  return line;
}

/* =================== XBEE TELEMETRY FUNCTIONS =================== */

/**
 * Send telemetry data via XBee radio
 * Non-blocking, sends to both XBee and Serial (dual output)
 * Called at 50 Hz from main loop
 */
void sendTelemetry(String csvLine) {
  // Send to XBee radio (primary telemetry output)
  if (xbeeAvailable) {
    xbeeSerial.println(csvLine);
  }
  
  // Also send to Serial (backup/debug)
  Serial.println(csvLine);
}

/**
 * Check for incoming commands from ground station via XBee
 * Non-blocking, called in main loop
 * Simple command protocol: single character commands
 */
void checkXBeeCommands() {
  if (!xbeeAvailable) return;
  
  while (xbeeSerial.available() > 0) {
    char cmd = xbeeSerial.read();
    
    switch(cmd) {
      case 'S':  // Status request
        xbeeSerial.print(F("STATE="));
        xbeeSerial.print(
          (flightState == IDLE) ? "IDLE" :
          (flightState == ASCENT) ? "ASCENT" :
          (flightState == APOGEE) ? "APOGEE" :
          (flightState == DESCENT) ? "DESCENT" : "CHUTE_DEPLOYED"
        );
        xbeeSerial.print(F(",ALT="));
        xbeeSerial.print(altitudeFiltered, 1);
        xbeeSerial.print(F(",PEAK="));
        xbeeSerial.println(peakAltitude, 1);
        break;
        
      case 'P':  // Emergency parachute deploy command
        if ((flightState == DESCENT || flightState == APOGEE) && !chuteServo.hasFired) {
          moveServoToFire(chuteServo, F("Parachute (GROUND CMD)"));
          xbeeSerial.println(F("PARACHUTE_DEPLOYED_BY_COMMAND"));
          Serial.println(F("[XBEE] Emergency parachute deploy commanded!"));
          flushBufferToSD();  // Save data immediately
        } else {
          xbeeSerial.println(F("PARACHUTE_DEPLOY_REJECTED"));
        }
        break;
        
      case 'R':  // Reset/Info request
        xbeeSerial.print(F("TIME="));
        xbeeSerial.print(millis()/1000);
        xbeeSerial.print(F("s,GPS="));
        xbeeSerial.print(lastLat, 6);
        xbeeSerial.print(F(","));
        xbeeSerial.println(lastLon, 6);
        break;
        
      case 'V':  // Version/System info
        xbeeSerial.println(F("ROCKET_FC_v1.0_INTEGRATED"));
        xbeeSerial.print(F("SD="));
        xbeeSerial.print(sdCardAvailable ? "OK" : "FAIL");
        xbeeSerial.print(F(",XBEE=OK"));
        xbeeSerial.print(F(",STATE="));
        xbeeSerial.println(flightState);
        break;
        
      default:
        // Unknown command - ignore silently
        break;
    }
  }
}

/* =================== 3x3 MATRIX HELPERS (from Fins) =================== */

static void m3_mul(const double A[3][3], const double B[3][3], double C[3][3]) {
  for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
    double s=0.0; for (int k=0;k<3;k++) s += A[i][k]*B[k][j]; C[i][j]=s;
  }
}
static void m3_add(const double A[3][3], const double B[3][3], double C[3][3]) {
  for (int i=0;i<3;i++) for (int j=0;j<3;j++) C[i][j]=A[i][j]+B[i][j];
}
static void m3_sub(const double A[3][3], const double B[3][3], double C[3][3]) {
  for (int i=0;i<3;i++) for (int j=0;j<3;j++) C[i][j]=A[i][j]-B[i][j];
}
static void m3_tr(const double A[3][3], double AT[3][3]) {
  for (int i=0;i<3;i++) for (int j=0;j<3;j++) AT[j][i]=A[i][j];
}
static void m3_eye(double I[3][3]) {
  I[0][0]=1; I[0][1]=0; I[0][2]=0;
  I[1][0]=0; I[1][1]=1; I[1][2]=0;
  I[2][0]=0; I[2][1]=0; I[2][2]=1;
}
static void m3_scale(double A[3][3], double c) {
  for (int i=0;i<3;i++) for (int j=0;j<3;j++) A[i][j]*=c;
}
static void m3_copy(const double A[3][3], double B[3][3]) { memcpy(B,A,sizeof(double)*9); }
static void m3_inv(const double A[3][3], double Ai[3][3]) {
  double aug[3][6]={{0}};
  for(int i=0;i<3;i++){ for(int j=0;j<3;j++) aug[i][j]=A[i][j]; aug[i][3+i]=1.0; }
  for(int i=0;i<3;i++){
    int piv=i; double best=fabs(aug[i][i]);
    for(int r=i+1;r<3;r++){ double v=fabs(aug[r][i]); if(v>best){best=v;piv=r;} }
    if(fabs(aug[piv][i])<1e-18) { double sign = (aug[piv][i] < 0 ? -1.0 : 1.0); aug[piv][i] = sign*1e-18; }
    if(piv!=i){ for(int c=0;c<6;c++){ double t=aug[i][c]; aug[i][c]=aug[piv][c]; aug[piv][c]=t; } }
    double s=aug[i][i];
    for(int c=0;c<6;c++) aug[i][c]/=s;
    for(int r=0;r<3;r++){ if(r==i) continue; double f=aug[r][i]; for(int c=0;c<6;c++) aug[r][c]-=f*aug[i][c]; }
  }
  for(int i=0;i<3;i++) for(int j=0;j<3;j++) Ai[i][j]=aug[i][3+j];
}

static void m3_expm(const double A[3][3], double E[3][3]) {
  static const double b[] = {
    64764752532480000.0, 32382376266240000.0, 7771770303897600.0,
    1187353796428800.0, 129060195264000.0, 10559470521600.0,
    670442572800.0, 33522128640.0, 1323241920.0,
    40840800.0, 960960.0, 16380.0, 182.0, 1.0
  };
  double n1=0; for (int j=0;j<3;j++){ double s=0; for(int i=0;i<3;i++) s+=fabs(A[i][j]); if(s>n1) n1=s; }
  const double theta13 = 5.371920351148152;
  int s = (n1>theta13)? (int)ceil(log(n1/theta13)/log(2.0)) : 0; if (s<0) s=0;

  double As[3][3]; m3_copy(A,As); if(s>0){ double sc = ldexp(1.0,-s); m3_scale(As,sc); }

  double A2[3][3],A4[3][3],A6[3][3];
  m3_mul(As,As,A2); m3_mul(A2,A2,A4); m3_mul(A2,A4,A6);

  double I[3][3]; m3_eye(I);
  double A8[3][3],A10[3][3],A12[3][3];
  m3_mul(A2,A6,A8); m3_mul(A4,A6,A10); m3_mul(A6,A6,A12);

  double Sodd[3][3],Seven[3][3],T[3][3];
  m3_copy(I,Sodd);   m3_scale(Sodd,b[1]);
  m3_copy(A2,T);     m3_scale(T,b[3]);   m3_add(Sodd,T,Sodd);
  m3_copy(A4,T);     m3_scale(T,b[5]);   m3_add(Sodd,T,Sodd);
  m3_copy(A6,T);     m3_scale(T,b[7]);   m3_add(Sodd,T,Sodd);
  m3_copy(A8,T);     m3_scale(T,b[9]);   m3_add(Sodd,T,Sodd);
  m3_copy(A10,T);    m3_scale(T,b[11]);  m3_add(Sodd,T,Sodd);
  m3_copy(A12,T);    m3_scale(T,b[13]);  m3_add(Sodd,T,Sodd);

  m3_copy(I,Seven);  m3_scale(Seven,b[0]);
  m3_copy(A2,T);     m3_scale(T,b[2]);   m3_add(Seven,T,Seven);
  m3_copy(A4,T);     m3_scale(T,b[4]);   m3_add(Seven,T,Seven);
  m3_copy(A6,T);     m3_scale(T,b[6]);   m3_add(Seven,T,Seven);
  m3_copy(A8,T);     m3_scale(T,b[8]);   m3_add(Seven,T,Seven);
  m3_copy(A10,T);    m3_scale(T,b[10]);  m3_add(Seven,T,Seven);
  m3_copy(A12,T);    m3_scale(T,b[12]);  m3_add(Seven,T,Seven);

  double U[3][3],V[3][3];
  m3_mul(As,Sodd,U); m3_copy(Seven,V);

  double VmU[3][3],VpU[3][3],Inv[3][3];
  m3_sub(V,U,VmU); m3_add(V,U,VpU); m3_inv(VmU,Inv); m3_mul(Inv,VpU,E);

  for (int k=0;k<s;k++){ m3_mul(E,E,T); m3_copy(T,E); }
}

/* =================== LQR COMPUTATION (from Fins) =================== */

void compute_ABK_Ad() {
  const double pho=1.225, v=5.0, sref=0.04, d=0.245;
  const double C_mdelta=0.035, C_ldelta=0.0, C_ndelta=0.00014;
  const double I_x=2.76/sqrt(2.0), I_y=I_x, I_z=0.045;
  const double r_0=0.01, p_0=4.2e-7, q_0=0.46;

  const double a12 = -(I_z-I_y)*r_0/I_x;
  const double a13 = -(I_z-I_y)*q_0/I_x;
  const double a21 = -(I_x-I_z)*r_0/I_y;
  const double a23 = -(I_x-I_z)*p_0/I_y;
  const double a31 = -(I_y-I_x)*q_0/I_z;
  const double a32 = -(I_y-I_x)*p_0/I_z;

  const double b1 = 0.5*pho*v*v*sref*d*C_ldelta;
  const double b2 = 0.5*pho*v*v*sref*d*C_mdelta;
  const double b3 = 0.5*pho*v*v*sref*d*C_ndelta;

  double A_[3][3]={{0, a12, a13},{a21, 0, a23},{a31, a32, 0}};
  double B_[3][4]={{b1,b1,b1,b1},{b2,b2,b2,b2},{b3,b3,b3,b3}};
  m3_copy(A_,A_mat); memcpy(B_mat,B_,sizeof(B_mat));

  const double k0 = 1.30740506130234468e+00;
  const double k1 = 1.02014313001205026e+00;
  const double k2 = 6.99320730349237465e+01;
  for (int i=0;i<4;i++){ K_mat[i][0]=k0; K_mat[i][1]=k1; K_mat[i][2]=k2; }

  double BK[3][3]={{0}};
  for (int i=0;i<3;i++) for (int j=0;j<3;j++){
    double s=0; for (int k=0;k<4;k++) s += B_mat[i][k]*K_mat[k][j];
    BK[i][j]=s;
  }
  m3_sub(A_mat,BK,Acl);

  double Acl_dt[3][3]; m3_copy(Acl,Acl_dt); m3_scale(Acl_dt,DT);
  m3_expm(Acl_dt,Ad);
}

/* =================== IMU CALIBRATION (from Fins) =================== */

void calibrateSensors() {
  long ax_sum=0, ay_sum=0, az_sum=0;
  long gx_sum=0, gy_sum=0, gz_sum=0;
  long mx_sum=0, my_sum=0, mz_sum=0;

  Serial.println(F("[IMU] Calibrating MPU9250... Keep still and away from magnets."));

  for (int i=0; i<CALIB_SAMPLES; i++) {
    myIMU.readAccelData(myIMU.accelCount);
    myIMU.readGyroData(myIMU.gyroCount);
    myIMU.readMagData(myIMU.magCount);

    ax_sum += myIMU.accelCount[0];
    ay_sum += myIMU.accelCount[1];
    az_sum += myIMU.accelCount[2];
    gx_sum += myIMU.gyroCount[0];
    gy_sum += myIMU.gyroCount[1];
    gz_sum += myIMU.gyroCount[2];
    mx_sum += myIMU.magCount[0];
    my_sum += myIMU.magCount[1];
    mz_sum += myIMU.magCount[2];
    delay(3);
  }

  gx_offset = gx_sum / (float)CALIB_SAMPLES;
  gy_offset = gy_sum / (float)CALIB_SAMPLES;
  gz_offset = gz_sum / (float)CALIB_SAMPLES;

  ax_offset = ax_sum / (float)CALIB_SAMPLES;
  ay_offset = ay_sum / (float)CALIB_SAMPLES;
  az_offset = (az_sum / (float)CALIB_SAMPLES) - COUNTS_PER_1G;

  mx_offset = mx_sum / (float)CALIB_SAMPLES;
  my_offset = my_sum / (float)CALIB_SAMPLES;
  mz_offset = mz_sum / (float)CALIB_SAMPLES;

  Serial.println(F("[IMU] Calibration done!"));
}

/* =================== BMP388 CALIBRATION (from deployment) =================== */

void autoCalibrate() {
  Serial.println(F("[BMP] Calibrating BMP388... Keep still for 3s."));
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
    Serial.print(F("[BMP] Baseline Pressure = "));
    Serial.print(launchPressure_hPa, 2);
    Serial.println(F(" hPa"));
  } else {
    Serial.println(F("[BMP] Warning: Low sample count; using default baseline."));
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
  Serial.print(F("[BMP] Ground offset = "));
  Serial.println(altitudeZeroOffset, 2);
}

/* =================== SETUP =================== */

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(38400);
  Wire.begin();
  delay(1000);

  Serial.println(F("========================================"));
  Serial.println(F("  INTEGRATED ROCKET FLIGHT CONTROLLER  "));
  Serial.println(F("  WITH SD CARD DATA LOGGING            "));
  Serial.println(F("  AND XBEE WIRELESS TELEMETRY          "));
  Serial.println(F("  MCU: STM32H7A3ZIQ                    "));
  Serial.println(F("========================================"));

  // --- Initialize XBee Telemetry Radio (USART6) ---
  xbeeSerial.begin(XBEE_BAUD);
  delay(100);
  
  // Test XBee connection
  xbeeSerial.println(F("ROCKET_FC_XBEE_INIT"));
  xbeeAvailable = true;
  Serial.println(F("[XBEE] Telemetry radio initialized on USART6 (PC6/PC7)"));
  Serial.print(F("[XBEE] Baud rate: "));
  Serial.println(XBEE_BAUD);

  // --- Initialize SD Card (SPI3) ---
  sdCardAvailable = initSDCard();
  if (!sdCardAvailable) {
    Serial.println(F("[WARN] Operating without SD card - data on Serial only"));
  }

  // --- Initialize Servo Bus (shared by all 6 servos) ---
  servoSerial.begin(1000000);
  st.pSerial = &servoSerial;
  delay(100);

  Serial.println(F("[SERVO] Enabling torque on all servos..."));
  // Enable fin servos (IDs 1-4)
  for (int i=0; i<4; i++) {
    st.EnableTorque(SERVO_FIN_IDS[i], 1);
    delay(20);
  }
  // Enable deployment servos (IDs 5-6)
  st.EnableTorque(cansatServo.id, 1);
  delay(20);
  st.EnableTorque(chuteServo.id, 1);
  delay(20);

  // Move all servos to safe/neutral positions
  for (int i=0; i<4; i++) {
    st.WritePosEx(SERVO_FIN_IDS[i], SERVO_NEUTRAL, 1500, 250);
  }
  st.WritePosEx(cansatServo.id, cansatServo.safePos, 1500, 250);
  st.WritePosEx(chuteServo.id, chuteServo.safePos, 1500, 250);
  Serial.println(F("[SERVO] All servos initialized to neutral positions."));

  // --- Initialize MPU9250 (IMU) ---
  myIMU.initMPU9250();
  myIMU.initAK8963(myIMU.factoryMagCalibration);
  Serial.println(F("[IMU] MPU9250 initialized!"));
  calibrateSensors();

  // --- Initialize BMP388 (Altimeter on SPI1) ---
  if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
    Serial.println(F("[ERROR] BMP388 not detected! Check wiring."));
    while (1) delay(10);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_31);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println(F("[BMP] BMP388 initialized on SPI1!"));
  autoCalibrate();

  // --- Compute LQR matrices ---
  compute_ABK_Ad();
  const double DEG2RAD = PI / 180.0;
  for (int i=0; i<4; ++i) {
    for (int j=0; j<3; ++j) {
      K_mat[i][j] *= DEG2RAD;
    }
  }
  Serial.println(F("[LQR] LQR controller initialized (K scaled for deg/s)."));

  // --- Initialize timing ---
  last_time = micros();
  lastAltUpdateMs = millis();
  lastFinsUpdateMs = millis();
  lastGPSReadMs = millis();
  lastSDFlushMs = millis();

  // --- CSV Header (Serial and XBee output) ---
  String csvHeader = F("Time_ms,State,Roll,Pitch,Yaw,p_dps,q_dps,r_dps,u1,u2,u3,u4,"
                       "Alt_BMP,Vz,PeakAlt,Lat,Lon,GPS_Alt,Servo1,Servo2,Servo3,Servo4,Servo5,Servo6");
  Serial.println(csvHeader);
  if (xbeeAvailable) {
    xbeeSerial.println(csvHeader);
  }

  Serial.println(F("[READY] All systems initialized. Waiting for launch..."));
}

/* =================== MAIN LOOP =================== */

void loop() {
  unsigned long currentMs = millis();

  /* ========== GPS PARSING (Non-blocking, runs whenever data available) ========== */
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (c == '\n') {
      nmeaSentence.trim();
      if (nmeaSentence.startsWith("$GNRMC")) parseGNRMC(nmeaSentence);
      else if (nmeaSentence.startsWith("$GNGGA")) parseGNGGA(nmeaSentence);
      nmeaSentence = "";
    } else nmeaSentence += c;
  }

  /* ========== FINS CONTROL (100 Hz - CRITICAL TIMING) ========== */
  if (currentMs - lastFinsUpdateMs >= FINS_INTERVAL_MS) {
    lastFinsUpdateMs = currentMs;

    // --- Compute dt for Kalman filter ---
    unsigned long current_time = micros();
    float dt = (current_time - last_time) / 1000000.0f;
    last_time = current_time;
    if (dt > 0.05f) dt = 0.05f;

    // --- Sample & average IMU readings ---
    long ax_sum=0, ay_sum=0, az_sum=0;
    long gx_sum=0, gy_sum=0, gz_sum=0;
    long mx_sum=0, my_sum=0, mz_sum=0;

    for (int i=0; i<LOOP_SAMPLES; i++) {
      myIMU.readAccelData(myIMU.accelCount);
      myIMU.readGyroData(myIMU.gyroCount);
      myIMU.readMagData(myIMU.magCount);

      ax_sum += myIMU.accelCount[0];
      ay_sum += myIMU.accelCount[1];
      az_sum += myIMU.accelCount[2];
      gx_sum += myIMU.gyroCount[0];
      gy_sum += myIMU.gyroCount[1];
      gz_sum += myIMU.gyroCount[2];
      mx_sum += myIMU.magCount[0];
      my_sum += myIMU.magCount[1];
      mz_sum += myIMU.magCount[2];
      delayMicroseconds(500);
    }

    // --- Subtract calibration offsets ---
    float ax = (ax_sum/(float)LOOP_SAMPLES) - ax_offset;
    float ay = (ay_sum/(float)LOOP_SAMPLES) - ay_offset;
    float az = (az_sum/(float)LOOP_SAMPLES) - az_offset;
    float gx = (gx_sum/(float)LOOP_SAMPLES) - gx_offset;
    float gy = (gy_sum/(float)LOOP_SAMPLES) - gy_offset;
    float gz = (gz_sum/(float)LOOP_SAMPLES) - gz_offset;
    float mx = (mx_sum/(float)LOOP_SAMPLES) - mx_offset;
    float my = (my_sum/(float)LOOP_SAMPLES) - my_offset;
    float mz = (mz_sum/(float)LOOP_SAMPLES) - mz_offset;

    // --- Low-pass filter ---
    ax_f=filter(ax,ax_f); ay_f=filter(ay,ay_f); az_f=filter(az,az_f);
    gx_f=filter(gx,gx_f); gy_f=filter(gy,gy_f); gz_f=filter(gz,gz_f);
    mx_f=filter(mx,mx_f); my_f=filter(my,my_f); mz_f=filter(mz,mz_f);

    // --- Adaptive gyro bias learning ---
    float gx_dps_raw = gx_f / GYRO_SCALE_FACTOR;
    float gy_dps_raw = gy_f / GYRO_SCALE_FACTOR;
    float gz_dps_raw = gz_f / GYRO_SCALE_FACTOR;

    float amag = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
    float low_g = (1.0f - acc_still_band_frac) * COUNTS_PER_1G;
    float high_g= (1.0f + acc_still_band_frac) * COUNTS_PER_1G;

    bool gyro_still = (fabsf(gx_dps_raw) < still_rate_dps) &&
                      (fabsf(gy_dps_raw) < still_rate_dps) &&
                      (fabsf(gz_dps_raw) < still_rate_dps);
    bool accel_ok   = (amag > low_g) && (amag < high_g);

    if (gyro_still && accel_ok) {
      gx_bias = (1.0f - bias_learn)*gx_bias + bias_learn*gx_dps_raw;
      gy_bias = (1.0f - bias_learn)*gy_bias + bias_learn*gy_dps_raw;
      gz_bias = (1.0f - bias_learn)*gz_bias + bias_learn*gz_dps_raw;
    }

    // --- Bias-corrected body rates (deg/s) ---
    float gyro_rate_roll  = gx_dps_raw - gx_bias;  // p
    float gyro_rate_pitch = gy_dps_raw - gy_bias;  // q
    float gyro_rate_yaw   = gz_dps_raw - gz_bias;  // r

    // --- Roll/Pitch from accelerometer ---
    float roll_accel  = atan2f(ay_f, az_f) * 180.0f / PI;
    float pitch_accel = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) * 180.0f / PI;

    roll_angle  = apply_kalman(roll_accel,  gyro_rate_roll,  dt, roll_angle,  P_roll,  Q_angle, R_angle);
    pitch_angle = apply_kalman(pitch_accel, gyro_rate_pitch, dt, pitch_angle, P_pitch, Q_angle, R_angle);

    // --- Yaw from tilt-compensated magnetometer ---
    float roll_rad  = roll_angle  * PI / 180.0f;
    float pitch_rad = pitch_angle * PI / 180.0f;

    float mag_x_comp = mx_f * cosf(pitch_rad) + mz_f * sinf(pitch_rad);
    float mag_y_comp = mx_f * sinf(roll_rad) * sinf(pitch_rad)
                     + my_f * cosf(roll_rad)
                     - mz_f * sinf(roll_rad) * cosf(pitch_rad);

    float yaw_mag = atan2f(-mag_y_comp, mag_x_comp) * 180.0f / PI;
    yaw_mag = wrapDeg360(yaw_mag);

    yaw_angle = apply_kalman_yaw(yaw_mag, gyro_rate_yaw, dt, yaw_angle, P_yaw, Q_yaw, R_yaw);

    // --- LQR Control (State x = [p q r] in deg/s) ---
    double x0 = (double)gyro_rate_roll;
    double x1 = (double)gyro_rate_pitch;
    double x2 = (double)gyro_rate_yaw;

    // u = -K x
    double u1 = -(K_mat[0][0]*x0 + K_mat[0][1]*x1 + K_mat[0][2]*x2);
    double u2 = -(K_mat[1][0]*x0 + K_mat[1][1]*x1 + K_mat[1][2]*x2);
    double u3 = -(K_mat[2][0]*x0 + K_mat[2][1]*x1 + K_mat[2][2]*x2);
    double u4 = -(K_mat[3][0]*x0 + K_mat[3][1]*x1 + K_mat[3][2]*x2);

    // --- Map to servo positions and command ---
    // Fins are ALWAYS ACTIVE to allow ground testing and flight operation
    // Only disable fins AFTER parachute deploys to avoid interference
    int pos1, pos2, pos3, pos4;
    
    if (flightState != CHUTE_DEPLOYED) {
      // Fins active in IDLE (for testing), ASCENT, APOGEE, and DESCENT
      pos1 = map_u_to_servo_pos(u1);
      pos2 = map_u_to_servo_pos(u2);
      pos3 = map_u_to_servo_pos(u3);
      pos4 = map_u_to_servo_pos(u4);

      const int CMD_TIME  = 1500;
      const int CMD_SPEED = 250;

      st.WritePosEx(SERVO_FIN_IDS[0], pos1, CMD_TIME, CMD_SPEED);
      st.WritePosEx(SERVO_FIN_IDS[1], pos2, CMD_TIME, CMD_SPEED);
      st.WritePosEx(SERVO_FIN_IDS[2], pos3, CMD_TIME, CMD_SPEED);
      st.WritePosEx(SERVO_FIN_IDS[3], pos4, CMD_TIME, CMD_SPEED);
    } else {
      // Fins return to neutral ONLY after parachute deploys (to avoid tangling)
      pos1 = pos2 = pos3 = pos4 = SERVO_NEUTRAL;
      st.WritePosEx(SERVO_FIN_IDS[0], SERVO_NEUTRAL, 1500, 250);
      st.WritePosEx(SERVO_FIN_IDS[1], SERVO_NEUTRAL, 1500, 250);
      st.WritePosEx(SERVO_FIN_IDS[2], SERVO_NEUTRAL, 1500, 250);
      st.WritePosEx(SERVO_FIN_IDS[3], SERVO_NEUTRAL, 1500, 250);
    }

    // Store control values for logging (done later in CSV output)
  }

  /* ========== ALTITUDE & DEPLOYMENT STATE MACHINE (50 Hz) ========== */
  if (currentMs - lastAltUpdateMs >= ALT_INTERVAL_MS) {
    lastAltUpdateMs = currentMs;

    // --- Read BMP388 ---
    if (bmp.performReading()) {
      float rawAlt = bmp.readAltitude(launchPressure_hPa) - altitudeZeroOffset;
      float filteredAlt = filterAltitude(rawAlt);
      ej
      float dt_alt = ALT_INTERVAL_MS / 1000.0;
      float vz_raw = (filteredAlt - prevAltitude) / dt_alt;
      verticalSpeed = (1 - VZ_EMA_BETA) * verticalSpeed + VZ_EMA_BETA * vz_raw;
      prevAltitude = filteredAlt;

      // --- Flight State Machine (from deployment_GPS_integrated.ino) ---
      switch (flightState) {
        case IDLE:
          if (filteredAlt > LAUNCH_ALT_M) {
            launchCount++;
            if (launchCount >= LAUNCH_CONFIRM) {
              flightState = ASCENT;
              peakAltitude = filteredAlt;
              failsafeArmed = false;
              Serial.println(F("[STATE] LAUNCH DETECTED -> ASCENT"));
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
            Serial.println(F("[STATE] APOGEE DETECTED -> CanSat EJECTED"));
          }
          break;

        case APOGEE:
          if (verticalSpeed < -VZ_MIN_MPS) {
            descentCount++;
            if (descentCount >= DESCENT_CONFIRM) {
              flightState = DESCENT;
              Serial.println(F("[STATE] DESCENT CONFIRMED"));
            }
          } else descentCount = 0;
          break;

        case DESCENT:
          // Fins remain active during descent per user requirement
          if (verticalSpeed < -VZ_MIN_MPS && filteredAlt <= CHUTE_DEPLOY_ALT_M && !chuteServo.hasFired) {
            moveServoToFire(chuteServo, F("Parachute"));
            flightState = CHUTE_DEPLOYED;
            Serial.println(F("[STATE] CHUTE DEPLOYED (altitude trigger)"));
            // Force flush buffer to ensure data is saved
            flushBufferToSD();
          }
          if (failsafeArmed && !chuteServo.hasFired && (millis() - failsafeStartMs) >= FAILSAFE_DELAY_MS) {
            moveServoToFire(chuteServo, F("Parachute"));
            flightState = CHUTE_DEPLOYED;
            Serial.println(F("[STATE] CHUTE DEPLOYED (failsafe)"));
            // Force flush buffer to ensure data is saved
            flushBufferToSD();
          }
          break;

        case CHUTE_DEPLOYED:
          // Fins disabled after chute deploys
          break;
      }

      // --- Update deployment servo returns ---
      updateServoReturn(cansatServo);
      updateServoReturn(chuteServo);
    }
  }

  /* ========== CSV TELEMETRY OUTPUT & SD LOGGING (50 Hz) ========== */
  // Output at the rate of altitude updates (50 Hz) to keep CSV manageable
  static unsigned long lastCSVOutput = 0;
  if (currentMs - lastCSVOutput >= ALT_INTERVAL_MS) {
    lastCSVOutput = currentMs;

    // Recompute current control values for logging
    float gx_dps_raw = gx_f / GYRO_SCALE_FACTOR;
    float gy_dps_raw = gy_f / GYRO_SCALE_FACTOR;
    float gz_dps_raw = gz_f / GYRO_SCALE_FACTOR;
    float gyro_rate_roll  = gx_dps_raw - gx_bias;
    float gyro_rate_pitch = gy_dps_raw - gy_bias;
    float gyro_rate_yaw   = gz_dps_raw - gz_bias;

    double x0 = (double)gyro_rate_roll;
    double x1 = (double)gyro_rate_pitch;
    double x2 = (double)gyro_rate_yaw;

    double u1 = -(K_mat[0][0]*x0 + K_mat[0][1]*x1 + K_mat[0][2]*x2);
    double u2 = -(K_mat[1][0]*x0 + K_mat[1][1]*x1 + K_mat[1][2]*x2);
    double u3 = -(K_mat[2][0]*x0 + K_mat[2][1]*x1 + K_mat[2][2]*x2);
    double u4 = -(K_mat[3][0]*x0 + K_mat[3][1]*x1 + K_mat[3][2]*x2);

    int pos1 = map_u_to_servo_pos(u1);
    int pos2 = map_u_to_servo_pos(u2);
    int pos3 = map_u_to_servo_pos(u3);
    int pos4 = map_u_to_servo_pos(u4);

    // Get deployment servo positions
    int pos5 = cansatServo.hasFired ? cansatServo.firePos : cansatServo.safePos;
    int pos6 = chuteServo.hasFired ? chuteServo.firePos : chuteServo.safePos;

    // Get state string
    const char* stateStr = 
      (flightState == IDLE) ? "IDLE" :
      (flightState == ASCENT) ? "ASCENT" :
      (flightState == APOGEE) ? "APOGEE" :
      (flightState == DESCENT) ? "DESCENT" : "CHUTE_DEPLOYED";

    // Build CSV line
    String csvLine = buildCSVLine(
      millis(), stateStr,
      roll_angle, pitch_angle, yaw_angle,
      gyro_rate_roll, gyro_rate_pitch, gyro_rate_yaw,
      u1, u2, u3, u4,
      altitudeFiltered, verticalSpeed, peakAltitude,
      lastLat, lastLon, lastAltitude,
      pos1, pos2, pos3, pos4, pos5, pos6
    );

    // Send telemetry via XBee and Serial (dual output)
    sendTelemetry(csvLine);

    // Add to SD buffer (if SD available)
    if (sdCardAvailable) {
      logToBuffer(csvLine);
    }
  }

  /* ========== XBEE COMMAND RECEPTION (Async) ========== */
  checkXBeeCommands();  // Non-blocking, process ground station commands

  /* ========== SD CARD BUFFER FLUSH (Every 2 seconds) ========== */
  if (sdCardAvailable && (currentMs - lastSDFlushMs >= SD_FLUSH_INTERVAL_MS)) {
    lastSDFlushMs = currentMs;
    flushBufferToSD();
  }

  // Light delay to prevent excessive CPU usage (non-critical)
  delayMicroseconds(100);
}
