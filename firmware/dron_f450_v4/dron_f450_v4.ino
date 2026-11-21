// ============================================================================
//  dron_f450_v4.ino — Firmware F450 (1 kg) profesional, modular
//
//  Hardware:
//    · ESP32-S3 + BNO086 + BME688 + GPS GP-02 + 4× ESC 2212-920KV @ 9450, 3S
//
//  Estructura modular (.h):
//    Config.h         — constantes
//    Types.h          — structs ESP-NOW
//    Filters.h        — primitivas de filtrado
//    PIDController.h  — PID profesional con d-on-meas, ff, i-relax, anti-windup
//    Attitude.h       — IMU (quat → gravedad → roll/pitch + gyro filtrado)
//    Baro.h           — BME688 con MovAvg + Biquad + LPF1
//    Gps.h            — TinyGPS+ con quality gating
//    Mixer.h          — X-quad con TPA + slew + reparto de saturación
//    RadioLink.h      — ESP-NOW RX/TX + failsafe
//    Diagnostics.h    — wizard único 'D' por serial
//
//  Tres modos: DISARMED · STABILIZE · LOITER (LOITER requiere GPS válido)
//  Failsafe en dos niveles: warning (300 ms) y cut (800 ms).
// ============================================================================
#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "Types.h"
#include "Filters.h"
#include "PIDController.h"
#include "Attitude.h"
#include "Baro.h"
#include "Gps.h"
#include "Mixer.h"
#include "RadioLink.h"
#include "Diagnostics.h"

// ─── Instancias globales ────────────────────────────────────────────────
Attitude   attitude;
Baro       baro;
Gps        gps;
Mixer      mixer;
RadioLink  radio;
Diagnostics diag(attitude, baro, gps, mixer, radio);

// PIDs
PIDController pidAngleRoll, pidAnglePitch;
PIDController pidRateRoll,  pidRatePitch, pidRateYaw;
PIDController pidAlt;

// RC filtrado
flt::LPF1  rcThrLpf, rcRollLpf, rcPitchLpf, rcYawLpf;

// Estado de vuelo
FlightMode currentMode = MODE_DISARMED;
bool       armed       = false;
uint32_t   armRequestT0= 0;
uint32_t   lastModeChgMs = 0;

// Altitud / hover hold
float      altSetpointM = 0.f;
bool       altHoldActive= false;
float      throttleHoverEstUs = HOVER_THROTTLE_US;

// In-air detection
uint32_t   inAirStartMs = 0;
bool       inAir = false;

// Anti-gravity
float      prevThrottleUs = 1000.f;

// Telemetría / status
uint32_t   nextLoopUs = 0;
uint32_t   nextBaroUs = 0;
uint32_t   nextTelemMs = 0;
uint32_t   nextStatusMs = 0;
uint32_t   lastLoopUs = 0;

// ── Helpers ─────────────────────────────────────────────────────────────
static inline float deadband(float v, float db) {
  if (v >  db) return v - db;
  if (v < -db) return v + db;
  return 0.f;
}
static inline float mapf(float x, float a, float b, float c, float d) {
  return c + (x - a) * (d - c) / (b - a);
}

// ── Configuración de los PID ────────────────────────────────────────────
void setupPIDs() {
  pidAngleRoll .setGains(ANGLE_KP_ROLL,  ANGLE_KI_ROLL,  ANGLE_KD, 0.f, ANGLE_ILIM);
  pidAnglePitch.setGains(ANGLE_KP_PITCH, ANGLE_KI_PITCH, ANGLE_KD, 0.f, ANGLE_ILIM);
  pidAngleRoll .iRelaxEnabled = false;
  pidAnglePitch.iRelaxEnabled = false;
  pidAngleRoll .setDFilter(0.f, LOOP_RATE_HZ);   // sin D en ángulo
  pidAnglePitch.setDFilter(0.f, LOOP_RATE_HZ);
  pidAngleRoll .outLimit  = MAX_RATE_FROM_ANGLE;
  pidAnglePitch.outLimit  = MAX_RATE_FROM_ANGLE;

  pidRateRoll .setGains(RATE_KP_ROLL,  RATE_KI_ROLL,  RATE_KD_ROLL,  RATE_KFF_ROLL,  RATE_ILIM_ROLL);
  pidRatePitch.setGains(RATE_KP_PITCH, RATE_KI_PITCH, RATE_KD_PITCH, RATE_KFF_PITCH, RATE_ILIM_PITCH);
  pidRateYaw  .setGains(YAW_KP,        YAW_KI,        YAW_KD,        YAW_KFF,        YAW_ILIM);
  pidRateRoll .setDFilter(D_TERM_LPF_FC_HZ, LOOP_RATE_HZ);
  pidRatePitch.setDFilter(D_TERM_LPF_FC_HZ, LOOP_RATE_HZ);
  pidRateYaw  .setDFilter(0.f, LOOP_RATE_HZ);
  pidRateRoll .setIRelax(true, I_RELAX_SETPOINT_DPS);
  pidRatePitch.setIRelax(true, I_RELAX_SETPOINT_DPS);
  pidRateRoll .outLimit = 500.f;
  pidRatePitch.outLimit = 500.f;
  pidRateYaw  .outLimit = 400.f;

  pidAlt.setGains(ALT_KP, ALT_KI, ALT_KD, 0.f, ALT_ILIM);
  pidAlt.setDFilter(2.0f, 20.0f);
  pidAlt.outLimit = 250.f;
  pidAlt.iRelaxEnabled = false;

  rcThrLpf.setCutoff(RC_LPF_FC_HZ, LOOP_RATE_HZ);
  rcRollLpf.setCutoff(RC_LPF_FC_HZ, LOOP_RATE_HZ);
  rcPitchLpf.setCutoff(RC_LPF_FC_HZ, LOOP_RATE_HZ);
  rcYawLpf.setCutoff(RC_LPF_FC_HZ, LOOP_RATE_HZ);
}

void resetIntegrators() {
  pidAngleRoll.reset();  pidAnglePitch.reset();
  pidRateRoll.reset();   pidRatePitch.reset();  pidRateYaw.reset();
  pidAlt.reset();
}

// ── Arming logic ────────────────────────────────────────────────────────
bool armingChecks(const DatosControl& rc) {
  if (!attitude.ready) return false;
  if (millis() - attitude.lastEventMs > 100) return false;
  if (rc.throttle > ARM_THROTTLE_MAX) return false;
  if (fabsf(attitude.rollDeg)  > ARM_MAX_TILT_DEG) return false;
  if (fabsf(attitude.pitchDeg) > ARM_MAX_TILT_DEG) return false;
  if (!radio.linkHealthy()) return false;
  return true;
}

void doArm() {
  resetIntegrators();
  attitude.resetYaw();
  altSetpointM   = baro.altitudeM;
  altHoldActive  = false;
  inAir          = false;
  inAirStartMs   = 0;
  prevThrottleUs = ESC_IDLE_US;
  armed = true;
  Serial.println(F("[FC] ARMADO"));
}
void doDisarm(const char* why) {
  armed = false;
  mixer.disarm();
  Serial.print(F("[FC] DESARMADO: ")); Serial.println(why);
}

// ── Setup ───────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println(F("================ F450 V4 BOOT ================"));

  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL, 400000);

  if (!attitude.begin()) Serial.println(F("[BOOT] IMU NO INICIA — vuelo deshabilitado"));
  if (!baro.begin())     Serial.println(F("[BOOT] BME688 NO INICIA — sin altitud"));
  baro.captureReference(30, 4000);
  gps.begin(Serial1, GPS_BAUD);
  if (!radio.begin())    Serial.println(F("[BOOT] ESP-NOW NO INICIA"));

  mixer.begin();
  mixer.disarm();
  setupPIDs();

  // Calibración inicial automática del bias del gyro (dron quieto al boot)
  delay(500);
  attitude.calibrateGyroBias(800, 4000);
  attitude.calibrateAttitudeOffset(400, 3000);

  Serial.print(F(" Pulsa '")); Serial.write(DIAG_PROMPT);
  Serial.println(F("' + ENTER en cualquier momento para el wizard de diagnóstico."));

  Serial.println(F("[BOOT] Listo — esperando enlace ESP-NOW y stick a 0."));
  uint32_t now = micros();
  nextLoopUs  = now;
  nextBaroUs  = now;
  lastLoopUs  = now;
  nextTelemMs = millis();
  nextStatusMs = millis();
}

// ── Tasks de menor frecuencia ───────────────────────────────────────────
void baroTask(uint32_t nowUs) {
  // El driver es asíncrono (beginReading/endReading); update() retorna true
  // sólo cuando una muestra se completó. Lo polleamos cada iteración del
  // bucle principal — nunca bloquea.
  static uint32_t lastCompleteUs = 0;
  if (baro.update()) {
    float dt = (lastCompleteUs == 0) ? 0.05f
                                     : (nowUs - lastCompleteUs) * 1e-6f;
    lastCompleteUs = nowUs;
    baro.updateVerticalSpeed(dt);
  }
}

void radioGpsTask() {
  gps.update();
}

void telemetryTask() {
  uint32_t now = millis();
  if (now < nextTelemMs) return;
  nextTelemMs = now + TELEMETRY_PERIOD_MS;
  TelemetriaDron t;
  t.imuListo   = attitude.ready;
  t.baroListo  = baro.ready;
  t.dronArmado = armed;
  radio.sendTelemetry(t);
}

void statusTask() {
  uint32_t now = millis();
  if (now < nextStatusMs) return;
  nextStatusMs = now + SERIAL_STATUS_MS;
  if (diag.isActive()) return;
  auto rc = radio.snapshot();
  Serial.printf("MODE=%u ARM=%d THR=%d  R/P/Y=%.1f/%.1f/%.1f  alt=%.2f  sats=%u link=%lums\n",
                (unsigned)currentMode, armed, rc.throttle,
                attitude.rollDeg, attitude.pitchDeg, attitude.yawDeg,
                baro.altitudeM, (unsigned)gps.satellites,
                (unsigned long)(millis() - radio.lastRxMs));
}

// ── Bucle de control (250 Hz) ───────────────────────────────────────────
void controlLoop(uint32_t nowUs) {
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  if (dt < 1e-4f) dt = LOOP_DT_S;
  if (dt > 0.05f)  dt = LOOP_DT_S;
  lastLoopUs = nowUs;

  attitude.update(12);

  DatosControl rc = radio.snapshot();
  bool linkOk = radio.linkHealthy();
  bool linkCut = radio.failsafeCut();

  // ── failsafe ──
  if (linkCut && armed) {
    doDisarm("FAILSAFE link cut");
  }

  // ── modo desde el remoto (con bloqueo de cambio) ──
  if (millis() - lastModeChgMs > MODE_CHANGE_LOCK_MS) {
    if (rc.flightMode != currentMode) {
      FlightMode req = (FlightMode)rc.flightMode;
      if (req == MODE_LOITER && !gps.qualityOk) {
        Serial.println(F("[FC] LOITER bloqueado: GPS sin calidad"));
      } else {
        currentMode = req;
        lastModeChgMs = millis();
        Serial.print(F("[FC] MODE -> ")); Serial.println((int)currentMode);
      }
    }
  }

  // ── arming / disarming ──
  if (rc.emergencyStop) { if (armed) doDisarm("E-STOP"); }
  else if (rc.armado && !armed) {
    if (armingChecks(rc)) {
      if (armRequestT0 == 0) armRequestT0 = millis();
      if (millis() - armRequestT0 > ARM_STABLE_MS) doArm();
    } else {
      armRequestT0 = 0;
    }
  } else if (!rc.armado && armed) {
    doDisarm("STICK disarm");
    armRequestT0 = 0;
  } else {
    armRequestT0 = 0;
  }

  if (!armed || !linkOk || currentMode == MODE_DISARMED) {
    mixer.disarm();
    return;
  }

  // ── RC filtrado + deadband + escalas ──
  float thrRaw   = constrain((float)rc.throttle, 1000.f, 2000.f);
  float rollRaw  = deadband((float)rc.roll,  STICK_DEADZONE_RAW);
  float pitchRaw = deadband((float)rc.pitch, STICK_DEADZONE_RAW);
  float yawRaw   = deadband((float)rc.yaw,   STICK_DEADZONE_RAW);

  float thrUs = rcThrLpf.update(thrRaw);
  float rollCmd  = rcRollLpf.update(rollRaw);
  float pitchCmd = rcPitchLpf.update(pitchRaw);
  float yawCmd   = rcYawLpf.update(yawRaw);

  // ── Bucle externo de ángulo ──
  float angLimit = (currentMode == MODE_LOITER) ? MAX_LOITER_TILT_DEG : MAX_ANGLE_CMD_DEG;
  float rollSP   = mapf(rollCmd,  -500.f, 500.f, -angLimit, angLimit);
  float pitchSP  = mapf(pitchCmd, -500.f, 500.f, -angLimit, angLimit);
  float yawRateSP= mapf(yawCmd,   -500.f, 500.f, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);

  bool groundLock = (thrUs < GROUND_THROTTLE_THR_US);

  // detección "in-air"
  if (thrUs > IN_AIR_THROTTLE_US) {
    if (inAirStartMs == 0) inAirStartMs = millis();
    if (!inAir && millis() - inAirStartMs > IN_AIR_HOLD_MS) inAir = true;
  } else {
    inAirStartMs = 0;
  }

  float rateRollSP  = pidAngleRoll .update(rollSP,  attitude.rollDeg,  dt, false, groundLock);
  float ratePitchSP = pidAnglePitch.update(pitchSP, attitude.pitchDeg, dt, false, groundLock);
  rateRollSP  = constrain(rateRollSP,  -MAX_RATE_FROM_ANGLE, MAX_RATE_FROM_ANGLE);
  ratePitchSP = constrain(ratePitchSP, -MAX_RATE_FROM_ANGLE, MAX_RATE_FROM_ANGLE);

  // ── Anti-gravity: throttle subiendo rápido → boost del integrador ──
  float dThr = (thrUs - prevThrottleUs) / dt;
  prevThrottleUs = thrUs;
  float iBoost = 1.0f;
  if (fabsf(dThr) > ANTI_GRAVITY_THR_DELTA / dt) iBoost = ANTI_GRAVITY_GAIN;

  bool antiWup = mixer.motorsSaturatedHigh;

  float pidR = pidRateRoll .update(rateRollSP,  attitude.gyroRollDps,  dt, antiWup, groundLock, iBoost);
  float pidP = pidRatePitch.update(ratePitchSP, attitude.gyroPitchDps, dt, antiWup, groundLock, iBoost);
  float pidY = pidRateYaw  .update(yawRateSP,   attitude.gyroYawDps,   dt, mixer.motorsSaturatedHigh, groundLock);

  // ── Hold/altitude opcional en LOITER ──
  float thrCmdUs = thrUs;
  if (currentMode == MODE_LOITER && baro.ready && inAir) {
    float thrDev = thrUs - HOVER_THROTTLE_US;
    if (fabsf(thrDev) < ALT_DEADBAND_US) {
      if (!altHoldActive) { altSetpointM = baro.altitudeM; altHoldActive = true; }
      float corr = pidAlt.update(altSetpointM, baro.altitudeM, dt, false, !inAir);
      thrCmdUs = HOVER_THROTTLE_US + corr;
    } else {
      altHoldActive = false;
      altSetpointM = baro.altitudeM;
    }
  }
  thrCmdUs = constrain(thrCmdUs, (float)ESC_IDLE_US, (float)ESC_MAX_US);

  // ── Detección de crash / flip → disarm automático ──
  if (inAir && (fabsf(attitude.rollDeg) > CRASH_ANGLE_DEG ||
                fabsf(attitude.pitchDeg) > CRASH_ANGLE_DEG)) {
    doDisarm("CRASH angle");
    return;
  }

  mixer.mix(thrCmdUs, pidR, pidP, pidY, armed, dt);
}

// ── Loop principal ──────────────────────────────────────────────────────
void loop() {
  diag.serviceTrigger();
  if (diag.isActive()) return;

  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextLoopUs) >= 0) {
    nextLoopUs += LOOP_PERIOD_US;
    controlLoop(nowUs);
  }
  baroTask(nowUs);
  radioGpsTask();
  telemetryTask();
  statusTask();
}
