/*
=====================================================================
Stabilized Rocket Controller (MPU9250 + 1D Kalman + LQR → SCServo)
---------------------------------------------------------------------
What this sketch does (high level):
1) Reads raw accel/gyro/mag samples from an MPU-9250 over I2C.
2) Calibrates and filters those signals, then estimates:
     - roll & pitch (deg) via accel+gyro fusion (simple 1D Kalman per axis)
     - yaw (deg) via tilt-compensated magnetometer + gyro fusion (1D Kalman)
3) Computes body-rate state x = [p, q, r] from gyro (deg/s).
4) Applies an LQR state-feedback law u = -K x  (K scaled to use deg/s).
5) Maps each u_i to a SCServo position command and sends it over UART.

IMPORTANT CHANGES you requested (already applied below):
- K is converted once at runtime to work with DEG/S states (Approach B).
- GYRO_SCALE_FACTOR updated to 131.0f (±250 dps default for MPU-9250).

Tuning knobs most people touch first:
- SCALE_U  : how strongly u_i moves the servo (counts per unit u)
- MAX_DELTA: clamp on how far from neutral a servo may move (safety)
- Q_angle/R_angle, Q_yaw/R_yaw: sensor fusion balance (Kalman-like)

Hardware notes:
- SCServo (Feetech SMS_STS) typically uses half-duplex UART @ 1 Mbps.
- The pins PA0/PA1 here are board-dependent; you may need another UART.
- Ensure your wiring supports half-duplex (or direction control if needed).

CSV logs:
- After each loop we print: time, p, q, r, u1..u4, servo1..servo4
  (p, q, r are in deg/s; u’s are unitless control efforts; servos in counts)

=====================================================================
*/

#include <Wire.h>
#include "MPU9250.h"
#include <SCServo.h>
#include <math.h>
#include <string.h>

#define MPU_ADDRESS 0x69
MPU9250 myIMU(MPU_ADDRESS, Wire, 400000); // I2C @ 400 kHz

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

// Adaptive gyro bias learner (deg/s domain)
float gx_bias=0.0f, gy_bias=0.0f, gz_bias=0.0f;
const float bias_learn = 0.0002f;      // learning rate (smaller = slower but smoother)
const float still_rate_dps = 2.5f;     // threshold to consider gyro “still” (deg/s)
const float acc_still_band_frac = 0.10f; // accel magnitude must be within ±10% of 1g

/* =================== Servo (SCServo) setup =================== */
SMS_STS st;

// NOTE: These pins are board-specific; verify they map to a valid UART.
// Many STM32 boards expose Serial2/Serial3 etc. You can switch to &Serial2.
HardwareSerial servoSerial(PA0, PA1);  // TX=PA1, RX=PA0 (half-duplex wiring required)

const int SERVO_IDS[4] = {1,2,3,4};  // addresses of the four servos
const int SERVO_NEUTRAL = 2048;      // center position (counts); SMS_STS range ~0..4095
const int MAX_DELTA = 500;           // max counts away from center (safety clamp)
const float SCALE_U = 200.0f;        // actuator mapping gain: counts per unit u (tune)

/* =================== Helper functions =================== */
/**
 * 1-pole low-pass with spike limiter.
 * Works directly in raw counts domain (OK as long as ranges remain consistent).
 */
float filter(float raw, float &filt) {
  float diff = raw - filt;
  if (fabsf(diff) > max_jump) {
    // cut a large step down to max_jump (preserve sign)
    raw = filt + (diff > 0 ? max_jump : -max_jump);
  }
  // low-pass update
  filt = alpha * raw + (1.0f - alpha) * filt;
  return filt;
}

/**
 * Very small scalar Kalman-like fusion for angle (roll/pitch):
 * Prediction with gyro, update with accel-derived angle.
 * angle, P are in/out state; Q/R are tunings.
 * Units: angle in deg, gyro_rate in deg/s, dt in s.
 */
float apply_kalman(float measured_angle, float gyro_rate, float dt,
                   float &angle, float &P, float Q, float R) {
  // Predict
  angle += gyro_rate * dt;
  P     += Q * dt;

  // Update
  float K = P / (P + R);
  angle   = angle + K * (measured_angle - angle);
  P       = (1.0f - K) * P;
  return angle;
}

// ---------- Yaw wrapping helpers (degrees) ----------
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

/**
 * Yaw fusion: predict yaw with gyro_z, update with tilt-compensated mag heading.
 * Both measured_yaw_deg and yaw_deg are in degrees.
 */
float apply_kalman_yaw(float measured_yaw_deg, float gyro_rate_dps, float dt,
                       float &yaw_deg, float &P, float Q, float R) {
  // Predict with gyro (deg)
  yaw_deg = wrapDeg360(yaw_deg + gyro_rate_dps * dt);
  P += Q * dt;

  // Update with magnetometer heading; innovation must be wrapped to (-180,180]
  float innov = wrapDeg180(measured_yaw_deg - yaw_deg);
  float K = P / (P + R);
  yaw_deg = wrapDeg360(yaw_deg + K * innov);
  P = (1.0f - K) * P;
  return yaw_deg;
}

/* =================== Calibration (static biases) =================== */
/**
 * Collects CALIB_SAMPLES while the device is still and:
 *  - Stores mean gyro counts as offsets (so later we subtract them)
 *  - Stores mean accel counts for X/Y; for Z we subtract 1g so that
 *    later az_corrected = raw - (mean - 1g) = (raw-mean) + 1g
 *  - Stores mean mag counts (NOTE: this is not a full mag calibration)
 */
void calibrateSensors() {
  long ax_sum=0, ay_sum=0, az_sum=0;
  long gx_sum=0, gy_sum=0, gz_sum=0;
  long mx_sum=0, my_sum=0, mz_sum=0;

  Serial.println("Calibrating sensors... Keep MPU still and away from magnets.");

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

  // Average biases (counts)
  gx_offset = gx_sum / (float)CALIB_SAMPLES;
  gy_offset = gy_sum / (float)CALIB_SAMPLES;
  gz_offset = gz_sum / (float)CALIB_SAMPLES;

  ax_offset = ax_sum / (float)CALIB_SAMPLES;
  ay_offset = ay_sum / (float)CALIB_SAMPLES;
  // Keep gravity for tilt math by adding back 1g
  az_offset = (az_sum / (float)CALIB_SAMPLES) - COUNTS_PER_1G;

  mx_offset = mx_sum / (float)CALIB_SAMPLES;
  my_offset = my_sum / (float)CALIB_SAMPLES;
  mz_offset = mz_sum / (float)CALIB_SAMPLES;

  Serial.println("Calibration done!");
}

/* =================== Small 3x3 matrix helpers + expm =================== */
// (Used to precompute model matrices and a discretized closed-loop Ad)
#define DT 0.01

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
  // Gauss–Jordan inverse with partial pivoting (3x3)
  double aug[3][6]={{0}};
  for(int i=0;i<3;i++){ for(int j=0;j<3;j++) aug[i][j]=A[i][j]; aug[i][3+i]=1.0; }
  for(int i=0;i<3;i++){
    int piv=i; double best=fabs(aug[i][i]);
    for(int r=i+1;r<3;r++){ double v=fabs(aug[r][i]); if(v>best){best=v;piv=r;} }
    // Numerical guard (avoid divide-by-near-zero)
    if(fabs(aug[piv][i])<1e-18) { double sign = (aug[piv][i] < 0 ? -1.0 : 1.0); aug[piv][i] = sign*1e-18; }
    if(piv!=i){ for(int c=0;c<6;c++){ double t=aug[i][c]; aug[i][c]=aug[piv][c]; aug[piv][c]=t; } }
    double s=aug[i][i];
    for(int c=0;c<6;c++) aug[i][c]/=s;
    for(int r=0;r<3;r++){ if(r==i) continue; double f=aug[r][i]; for(int c=0;c<6;c++) aug[r][c]-=f*aug[i][c]; }
  }
  for(int i=0;i<3;i++) for(int j=0;j<3;j++) Ai[i][j]=aug[i][3+j];
}

// Higham’s Pade(13) expm (3x3 specialization)
static void m3_expm(const double A[3][3], double E[3][3]) {
  static const double b[] = {
    64764752532480000.0, 32382376266240000.0, 7771770303897600.0,
    1187353796428800.0, 129060195264000.0, 10559470521600.0,
    670442572800.0, 33522128640.0, 1323241920.0,
    40840800.0, 960960.0, 16380.0, 182.0, 1.0
  };
  // 1-norm (col sum)
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

// -------- LQR matrices (hard-coded K from your offline design) --------
double A_mat[3][3], B_mat[3][4], K_mat[4][3], Acl[3][3], Ad[3][3];

/**
 * Builds continuous-time A, B from your parameters, then sets K.
 * Also precomputes Acl = A - B*K and Ad = expm(Acl*DT) (Ad not used in loop).
 */
void compute_ABK_Ad() {
  const double pho=1.225, v=5.0, sref=0.04, d=0.245;
  const double C_mdelta=0.035, C_ldelta=0.0, C_ndelta=0.00014;
  const double I_x=2.76/sqrt(2.0), I_y=I_x, I_z=0.045;
  const double r_0=0.01, p_0=4.2e-7, q_0=0.46;

  // Rigid-body cross-coupling terms (units: 1/s)
  const double a12 = -(I_z-I_y)*r_0/I_x;
  const double a13 = -(I_z-I_y)*q_0/I_x;
  const double a21 = -(I_x-I_z)*r_0/I_y;
  const double a23 = -(I_x-I_z)*p_0/I_y;
  const double a31 = -(I_y-I_x)*q_0/I_z;
  const double a32 = -(I_y-I_x)*p_0/I_z;

  // Aerodynamic control gains into rotational dynamics (simplified, symmetric actuators)
  const double b1 = 0.5*pho*v*v*sref*d*C_ldelta;
  const double b2 = 0.5*pho*v*v*sref*d*C_mdelta;
  const double b3 = 0.5*pho*v*v*sref*d*C_ndelta;

  double A_[3][3]={{0, a12, a13},{a21, 0, a23},{a31, a32, 0}};
  double B_[3][4]={{b1,b1,b1,b1},{b2,b2,b2,b2},{b3,b3,b3,b3}};
  m3_copy(A_,A_mat); memcpy(B_mat,B_,sizeof(B_mat));

  // Hard-coded LQR gains (designed for SI rad/s). We will convert to deg/s later.
  const double k0 = 1.30740506130234468e+00;
  const double k1 = 1.02014313001205026e+00;
  const double k2 = 6.99320730349237465e+01;
  for (int i=0;i<4;i++){ K_mat[i][0]=k0; K_mat[i][1]=k1; K_mat[i][2]=k2; }

  // Closed-loop Acl = A - B*K (not used in the loop directly, but computed for completeness)
  double BK[3][3]={{0}};
  for (int i=0;i<3;i++) for (int j=0;j<3;j++){
    double s=0; for (int k=0;k<4;k++) s += B_mat[i][k]*K_mat[k][j];
    BK[i][j]=s;
  }
  m3_sub(A_mat,BK,Acl);

  // Discrete closed-loop transition for DT (not used directly)
  double Acl_dt[3][3]; m3_copy(Acl,Acl_dt); m3_scale(Acl_dt,DT);
  m3_expm(Acl_dt,Ad);
}

/* =================== Helper: map controller u -> servo position =================== */
/**
 * Converts unitless control effort u to a servo command (counts).
 * 1) scale by SCALE_U  (counts per unit u)
 * 2) clamp to ±MAX_DELTA around SERVO_NEUTRAL
 * 3) clamp to servo’s absolute range [0..4095]
 */
int map_u_to_servo_pos(double u) {
  int delta = (int)round(u * SCALE_U);
  if (delta >  MAX_DELTA) delta =  MAX_DELTA;
  if (delta < -MAX_DELTA) delta = -MAX_DELTA;

  int pos = SERVO_NEUTRAL + delta;
  if (pos < 0)    pos = 0;
  if (pos > 4095) pos = 4095;
  return pos;
}

/* =================== Setup =================== */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  // IMU init (uses library defaults: typically ±2g, ±250 dps)
  myIMU.initMPU9250();
  myIMU.initAK8963(myIMU.factoryMagCalibration);

  Serial.println("MPU9250 initialized!");
  calibrateSensors();          // compute static offsets (keep still!)
  last_time = micros();

  compute_ABK_Ad();            // build A,B and set K (in rad/s design)

  // -------- Convert K to work with DEG/S states (Approach B) --------
  // If K was designed for rad/s, but we feed p,q,r in deg/s, we must scale K by (π/180).
  const double DEG2RAD = PI / 180.0;
  for (int i=0; i<4; ++i) {
    for (int j=0; j<3; ++j) {
      K_mat[i][j] *= DEG2RAD; // K_deg = K_rad * (π/180)
    }
  }
  // ------------------------------------------------------------------

  // Servo bus init (1 Mbps); ensure wiring/uart supports SCServo half-duplex
  servoSerial.begin(1000000);
  st.pSerial = &servoSerial;
  delay(100);

  Serial.println("Enabling torque on servos...");
  for (int i=0; i<4; i++) {
    st.EnableTorque(SERVO_IDS[i], 1);
    delay(20);
  }

  // CSV header (p,q,r are deg/s; u’s are unitless; servos are counts)
  Serial.println("time,p_dps,q_dps,r_dps,u1,u2,u3,u4,servo1,servo2,servo3,servo4");
}

/* =================== Main loop: IMU → Fusion → LQR → Servos =================== */
void loop() {
  // ---- dt computation (capped for robustness) ----
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0f;
  last_time = current_time;
  if (dt > 0.05f) dt = 0.05f;   // avoid huge dt on hiccups

  // ---- sample & average a few reads (simple anti-noise) ----
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
    delay(2);
  }

  // ---- subtract DC offsets (calibration) ----
  float ax = (ax_sum/(float)LOOP_SAMPLES) - ax_offset;
  float ay = (ay_sum/(float)LOOP_SAMPLES) - ay_offset;
  float az = (az_sum/(float)LOOP_SAMPLES) - az_offset;
  float gx = (gx_sum/(float)LOOP_SAMPLES) - gx_offset;
  float gy = (gy_sum/(float)LOOP_SAMPLES) - gy_offset;
  float gz = (gz_sum/(float)LOOP_SAMPLES) - gz_offset;
  float mx = (mx_sum/(float)LOOP_SAMPLES) - mx_offset;
  float my = (my_sum/(float)LOOP_SAMPLES) - my_offset;
  float mz = (mz_sum/(float)LOOP_SAMPLES) - mz_offset;

  // ---- low-pass filter (with spike limiting) ----
  ax_f=filter(ax,ax_f); ay_f=filter(ay,ay_f); az_f=filter(az,az_f);
  gx_f=filter(gx,gx_f); gy_f=filter(gy,gy_f); gz_f=filter(gz,gz_f);
  mx_f=filter(mx,mx_f); my_f=filter(my,my_f); mz_f=filter(mz,mz_f);

  // ---- adaptive gyro bias learning (deg/s domain) ----
  float gx_dps_raw = gx_f / GYRO_SCALE_FACTOR;
  float gy_dps_raw = gy_f / GYRO_SCALE_FACTOR;
  float gz_dps_raw = gz_f / GYRO_SCALE_FACTOR;

  float amag = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f); // stillness check
  float low_g = (1.0f - acc_still_band_frac) * COUNTS_PER_1G;
  float high_g= (1.0f + acc_still_band_frac) * COUNTS_PER_1G;

  bool gyro_still = (fabsf(gx_dps_raw) < still_rate_dps) &&
                    (fabsf(gy_dps_raw) < still_rate_dps) &&
                    (fabsf(gz_dps_raw) < still_rate_dps);
  bool accel_ok   = (amag > low_g) && (amag < high_g);

  if (gyro_still && accel_ok) {
    // Exponential moving average of biases
    gx_bias = (1.0f - bias_learn)*gx_bias + bias_learn*gx_dps_raw;
    gy_bias = (1.0f - bias_learn)*gy_bias + bias_learn*gy_dps_raw;
    gz_bias = (1.0f - bias_learn)*gz_bias + bias_learn*gz_dps_raw;
  }

  // Bias-corrected body rates (deg/s)
  float gyro_rate_roll  = gx_dps_raw - gx_bias;  // p
  float gyro_rate_pitch = gy_dps_raw - gy_bias;  // q
  float gyro_rate_yaw   = gz_dps_raw - gz_bias;  // r

  // ---- roll/pitch from accelerometer + gyro (deg) ----
  float roll_accel  = atan2f(ay_f, az_f) * 180.0f / PI;
  float pitch_accel = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) * 180.0f / PI;

  roll_angle  = apply_kalman(roll_accel,  gyro_rate_roll,  dt, roll_angle,  P_roll,  Q_angle, R_angle);
  pitch_angle = apply_kalman(pitch_accel, gyro_rate_pitch, dt, pitch_angle, P_pitch, Q_angle, R_angle);

  // ---- yaw from tilt-compensated magnetometer + gyro (deg) ----
  float roll_rad  = roll_angle  * PI / 180.0f;
  float pitch_rad = pitch_angle * PI / 180.0f;

  // Tilt compensation (simple form; assumes board frame aligns with sensor frame)
  float mag_x_comp = mx_f * cosf(pitch_rad) + mz_f * sinf(pitch_rad);
  float mag_y_comp = mx_f * sinf(roll_rad) * sinf(pitch_rad)
                   + my_f * cosf(roll_rad)
                   - mz_f * sinf(roll_rad) * cosf(pitch_rad);

  float yaw_mag = atan2f(-mag_y_comp, mag_x_comp) * 180.0f / PI;
  yaw_mag = wrapDeg360(yaw_mag);

  yaw_angle = apply_kalman_yaw(yaw_mag, gyro_rate_yaw, dt, yaw_angle, P_yaw, Q_yaw, R_yaw);

  // ---- Debug print of angles (human-readable) ----
  Serial.print("Roll: ");  Serial.print(roll_angle,1);
  Serial.print(" | Pitch: "); Serial.print(pitch_angle,1);
  Serial.print(" | Yaw: ");   Serial.print(yaw_angle,1);
  Serial.print(" | dt: ");    Serial.print(dt*1000.0f,1);
  Serial.println(" ms");

  // ---- LQR control (K already scaled to expect deg/s) ----
  // State x = [p q r] in deg/s
  double x0 = (double)gyro_rate_roll;
  double x1 = (double)gyro_rate_pitch;
  double x2 = (double)gyro_rate_yaw;

  // u = -K x  (four identical rows of K apply to each actuator)
  double u1 = -(K_mat[0][0]*x0 + K_mat[0][1]*x1 + K_mat[0][2]*x2);
  double u2 = -(K_mat[1][0]*x0 + K_mat[1][1]*x1 + K_mat[1][2]*x2);
  double u3 = -(K_mat[2][0]*x0 + K_mat[2][1]*x1 + K_mat[2][2]*x2);
  double u4 = -(K_mat[3][0]*x0 + K_mat[3][1]*x1 + K_mat[3][2]*x2);

  // ---- Map u_i to servo positions and command the actuators ----
  int pos1 = map_u_to_servo_pos(u1);
  int pos2 = map_u_to_servo_pos(u2);
  int pos3 = map_u_to_servo_pos(u3);
  int pos4 = map_u_to_servo_pos(u4);

  // Feetech SMS_STS: WritePosEx(ID, position, time_ms, speed)
  // time: motion duration; speed: 0 (implementation-defined, often "no limit")
  const int CMD_TIME  = 1500;  // ms (very snappy; increase if your linkages chatter)
  const int CMD_SPEED = 250;

  st.WritePosEx(SERVO_IDS[0], pos1, CMD_TIME, CMD_SPEED);
  st.WritePosEx(SERVO_IDS[1], pos2, CMD_TIME, CMD_SPEED);
  st.WritePosEx(SERVO_IDS[2], pos3, CMD_TIME, CMD_SPEED);
  st.WritePosEx(SERVO_IDS[3], pos4, CMD_TIME, CMD_SPEED);

  // ---- CSV output: time, p,q,r (deg/s), u1..u4 (unitless), servo positions (counts) ----
  double tsec = millis()/1000.0;
  Serial.print(tsec,6); Serial.print(",");
  Serial.print(x0,6);  Serial.print(",");
  Serial.print(x1,6);  Serial.print(",");
  Serial.print(x2,6);  Serial.print(",");
  Serial.print(u1,6);  Serial.print(",");
  Serial.print(u2,6);  Serial.print(",");
  Serial.print(u3,6);  Serial.print(",");
  Serial.print(u4,6);  Serial.print(",");
  Serial.print(pos1);  Serial.print(",");
  Serial.print(pos2);  Serial.print(",");
  Serial.print(pos3);  Serial.print(",");
  Serial.println(pos4);

  // Light pacing; most timing comes from sensor I/O & servo bus
  delay(2);
}