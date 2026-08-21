/*
 * ovd_uart_nrf52840.ino
 * ------------------------------------------------------------------
 * Own-Voice-Detection on XIAO nRF52840 (Sense, onboard LSM6DS3TR-C),
 * with a scalar KALMAN FILTER on the accelerometer + optional
 * HAPTIC BLANKING to reject motor-induced vibration.
 * Sends 1/0 over UART (Serial1, D6=TX) to the ESP32-S3.
 *
 * ABOUT NOISE:
 *   - The Kalman filter removes RANDOM sensor noise -> cleaner reads.
 *   - It CANNOT remove haptic vibration: an ERM spins ~100-250 Hz,
 *     inside the speech band, so it looks like real signal. To reject
 *     it, wire a "motors active" line from the ESP32 to HAPTIC_BLANK_PIN
 *     (HIGH while motors fire); detection is frozen during that time.
 *
 * BUILD FROM A SPACE-FREE PATH (Documents/Arduino), or the nRF52
 * arm-none-eabi compiler errors on the space.
 *
 * Board: Seeed nRF52 Boards -> XIAO nRF52840 Sense
 * Library: Seeed Arduino LSM6DS3
 * ------------------------------------------------------------------
 */

#include "LSM6DS3.h"
#include "Wire.h"

LSM6DS3 imu(I2C_MODE, 0x6A);

const uint32_t UART_BAUD = 115200;

// ---- detection tunables ----
const float    HP_CUTOFF_HZ  = 80.0f;
const float    ENV_TAU_S     = 0.030f;
const float    VIB_THRESHOLD = 0.010f;   // g
const float    GYRO_GATE_DPS = 40.0f;    // deg/s
const int      GYRO_EVERY    = 16;
const uint32_t HEARTBEAT_MS  = 50;

// ---- Kalman filter (scalar, one per accel axis) ----
// Q = process noise (higher = more responsive / less smoothing)
// R = measurement noise (higher = more smoothing)
// Tune the Q/R ratio: raise Q if speech gets smeared, raise R if the
// signal is still jittery. These defaults filter lightly to keep speech.
const float KAL_Q = 0.05f;
const float KAL_R = 0.01f;
struct Kalman1D { float x; float P; };
Kalman1D kAx = {0, 1}, kAy = {0, 1}, kAz = {0, 1};

float kalman(Kalman1D &k, float meas) {
  k.P += KAL_Q;                       // predict: uncertainty grows
  float K = k.P / (k.P + KAL_R);      // Kalman gain
  k.x += K * (meas - k.x);            // update estimate toward measurement
  k.P *= (1.0f - K);                  // shrink uncertainty
  return k.x;
}

// ---- optional haptic blanking ----
// Wire the ESP32's "motors active" signal here (HIGH = motors on).
// While HIGH, the accelerometer is untrustworthy, so detection is held.
// Set to -1 to disable the feature.
const int HAPTIC_BLANK_PIN = -1;      // e.g. set to D3 once wired; -1 = off

// ---- state ----
float hpX = 0, hpY = 0, hpZ = 0, pX = 0, pY = 0, pZ = 0;
float env = 0, gyroMag = 0;
int   gyroCtr = 0;
uint32_t lastMicros = 0, lastSend = 0;
float measuredHz = 800.0f;
bool  lastState = false;

void setup() {
  Serial.begin(115200);        // USB debug
  Serial1.begin(UART_BAUD);    // UART out to ESP32 (D6=TX)

  if (HAPTIC_BLANK_PIN >= 0) pinMode(HAPTIC_BLANK_PIN, INPUT);

  imu.settings.accelEnabled    = 1;
  imu.settings.accelRange      = 4;
  imu.settings.accelSampleRate = 1660;
  imu.settings.accelBandWidth  = 400;
  imu.settings.gyroEnabled     = 1;
  imu.settings.gyroRange       = 500;
  imu.settings.gyroSampleRate  = 1660;

  if (imu.begin() != 0) Serial.println("ERROR: IMU not found (XIAO nRF52840 Sense?)");
  else                  Serial.println("OVD + Kalman + UART ready.");

  lastMicros = micros();
  lastSend   = millis();
}

void loop() {
  // ---- timing ----
  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / measuredHz;
  measuredHz = 0.98f * measuredHz + 0.02f * (1.0f / dt);

  // ---- read accel + Kalman-filter each axis (removes random noise) ----
  float ax = kalman(kAx, imu.readFloatAccelX());
  float ay = kalman(kAy, imu.readFloatAccelY());
  float az = kalman(kAz, imu.readFloatAccelZ());

  // ---- haptic blanking: freeze detection while motors are firing ----
  bool blanked = (HAPTIC_BLANK_PIN >= 0) && (digitalRead(HAPTIC_BLANK_PIN) == HIGH);

  bool speaking;
  if (!blanked) {
    // high-pass each (Kalman-cleaned) axis
    float rc    = 1.0f / (2.0f * PI * HP_CUTOFF_HZ);
    float alpha = rc / (rc + dt);

    hpX = alpha * (hpX + ax - pX); pX = ax;
    hpY = alpha * (hpY + ay - pY); pY = ay;
    hpZ = alpha * (hpZ + az - pZ); pZ = az;

    float vibAC = sqrtf(hpX * hpX + hpY * hpY + hpZ * hpZ);

    float k = dt / ENV_TAU_S;
    if (k > 1.0f) k = 1.0f;
    env += k * (vibAC - env);

    if (++gyroCtr >= GYRO_EVERY) {
      gyroCtr = 0;
      float gx = imu.readFloatGyroX();
      float gy = imu.readFloatGyroY();
      float gz = imu.readFloatGyroZ();
      gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
    }

    speaking = (env > VIB_THRESHOLD) && (gyroMag < GYRO_GATE_DPS);
  } else {
    // motors firing: accel is corrupted -> keep the high-pass baseline
    // current (so we don't get a jump when blanking releases) and hold
    // the previous decision instead of trusting the noisy signal.
    pX = ax; pY = ay; pZ = az;
    speaking = lastState;
  }

  // ---- send over UART: on change immediately, else heartbeat ----
  uint32_t nowMs = millis();
  if (speaking != lastState || (nowMs - lastSend) >= HEARTBEAT_MS) {
    lastState = speaking;
    lastSend  = nowMs;

    Serial1.println(speaking ? "1" : "0");

    Serial.print("env=");   Serial.print(env, 4);
    Serial.print(" gyro="); Serial.print(gyroMag, 1);
    Serial.print(blanked ? " [BLANKED]" : "");
    Serial.println(speaking ? "  -> 1 (SPEAKING)" : "  -> 0");
  }
}
