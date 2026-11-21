// ============================================================================
//  Attitude.h — Estimación de actitud y velocidad angular
//
//  Estrategia matemática:
//   1. La BNO086 entrega un cuaternión gravitacional (GAME_ROTATION_VECTOR).
//      De él extraemos el vector "up" en el frame del sensor mediante la
//      tercera fila de la matriz de rotación (sin necesidad de Euler).
//   2. Aplicamos un remap de ejes (sensor → dron) para obtener el vector up
//      en el frame del cuerpo del dron (X adelante, Y derecha, Z abajo).
//   3. Roll y Pitch salen directamente de up_body por geometría:
//          pitch = asin(up_x)
//          roll  = atan2(-up_y, -up_z)
//      Este enfoque es estable frente a cualquier orientación de montaje y
//      no depende de "imuRollOffset" como en V3.3 (el vector gravedad se
//      ancla por sí solo y los offsets capturan únicamente la inclinación
//      mecánica de la placa respecto al chasis).
//   4. Yaw se integra del giroscopio (no hay magnetómetro; suficiente para
//      vuelos cortos y para LOITER). Reset al armar.
//   5. Los datos del gyro pasan por: bias removal → mediana3 → notch (opc)
//      → biquad LPF → guardado.
//   6. Los ángulos pasan por: spike filter → biquad LPF.
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include "Config.h"
#include "Filters.h"

class Attitude {
public:
  // ── Estado público (en frame del dron) ────────────────────────────────
  float rollDeg  = 0.f;
  float pitchDeg = 0.f;
  float yawDeg   = 0.f;             // integrado del giroscopio (0 al armar)
  float gyroRollDps  = 0.f;
  float gyroPitchDps = 0.f;
  float gyroYawDps   = 0.f;
  bool  ready = false;
  unsigned long lastEventMs = 0;

  // ── Espejos públicos para diagnóstico ─────────────────────────────────
  uint32_t totalSamples   = 0;       // eventos procesados (quat+gyro)
  float    gyroBiasRoll   = 0.f;     // bias mapeado a body frame (dps)
  float    gyroBiasPitch  = 0.f;
  float    gyroBiasYaw    = 0.f;
  float    attOffRoll     = 0.f;     // offset mecánico (deg)
  float    attOffPitch    = 0.f;

  // ── Init ───────────────────────────────────────────────────────────────
  bool begin() {
    if      (imu_.begin_I2C(0x4B, &Wire)) Serial.println("[IMU] BNO086 @ 0x4B");
    else if (imu_.begin_I2C(0x4A, &Wire)) Serial.println("[IMU] BNO086 @ 0x4A");
    else { Serial.println("[IMU] ERROR: no encontrada"); return false; }

    imu_.enableReport(SH2_GAME_ROTATION_VECTOR, 4000);     // 250 Hz
    imu_.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 4000);

    // Filtros (configurados a la frecuencia nominal del IMU = 250 Hz)
    bqGR_.configure(GYRO_BIQUAD_FC_HZ,  LOOP_RATE_HZ);
    bqGP_.configure(GYRO_BIQUAD_FC_HZ,  LOOP_RATE_HZ);
    bqGY_.configure(GYRO_BIQUAD_FC_HZ,  LOOP_RATE_HZ);
    bqRoll_.configure (ANGLE_BIQUAD_FC_HZ, LOOP_RATE_HZ);
    bqPitch_.configure(ANGLE_BIQUAD_FC_HZ, LOOP_RATE_HZ);
    notchGR_.configure(GYRO_NOTCH_FC_HZ, LOOP_RATE_HZ, GYRO_NOTCH_Q);
    notchGP_.configure(GYRO_NOTCH_FC_HZ, LOOP_RATE_HZ, GYRO_NOTCH_Q);
    notchGY_.configure(GYRO_NOTCH_FC_HZ, LOOP_RATE_HZ, GYRO_NOTCH_Q);

    spkRoll_  = flt::SpikeFilter(SPIKE_THR_ANGLE_DEG);
    spkPitch_ = flt::SpikeFilter(SPIKE_THR_ANGLE_DEG);
    spkGR_    = flt::SpikeFilter(SPIKE_THR_GYRO_DPS);
    spkGP_    = flt::SpikeFilter(SPIKE_THR_GYRO_DPS);
    spkGY_    = flt::SpikeFilter(SPIKE_THR_GYRO_DPS);

    delay(50);
    lastEventMs = millis();
    return true;
  }

  // ── Calibración de bias del giroscopio (dron quieto) ──────────────────
  bool calibrateGyroBias(int targetSamples = 800, unsigned long timeoutMs = 10000) {
    Serial.printf("[IMU] Calibrando bias gyro (%d muestras)... mantén quieto.\n",
                  targetSamples);
    float sx = 0, sy = 0, sz = 0;
    int n = 0;
    unsigned long t0 = millis();
    while (n < targetSamples && (millis() - t0) < timeoutMs) {
      if (imu_.wasReset()) reenableReports_();
      if (imu_.getSensorEvent(&ev_) && ev_.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
        sx += ev_.un.gyroscopeUncal.x;
        sy += ev_.un.gyroscopeUncal.y;
        sz += ev_.un.gyroscopeUncal.z;
        n++;
      }
      delay(1);
    }
    if (n < 100) {
      Serial.printf("[IMU] WARN: solo %d muestras, bias=0\n", n);
      biasX_ = biasY_ = biasZ_ = 0.f;
      return false;
    }
    biasX_ = sx / n; biasY_ = sy / n; biasZ_ = sz / n;
    // mirror bias en body frame (dps) para diagnóstico
    constexpr float RAD2DEG = 57.29577951f;
    gyroBiasRoll  = GYRO_SIGN_ROLL  * biasY_ * RAD2DEG;
    gyroBiasPitch = GYRO_SIGN_PITCH * biasZ_ * RAD2DEG;
    gyroBiasYaw   = GYRO_SIGN_YAW   * biasX_ * RAD2DEG;
    Serial.printf("[IMU] Bias gyro (rad/s) X:%.5f Y:%.5f Z:%.5f (n=%d)\n",
                  biasX_, biasY_, biasZ_, n);
    return true;
  }

  // ── Calibración del offset mecánico de actitud (dron nivelado) ────────
  bool calibrateAttitudeOffset(int targetSamples = 400, unsigned long timeoutMs = 8000) {
    Serial.printf("[IMU] Capturando nivel (%d muestras)... no muevas el dron.\n",
                  targetSamples);
    double sumR = 0, sumP = 0;
    int n = 0;
    unsigned long t0 = millis();
    while (n < targetSamples && (millis() - t0) < timeoutMs) {
      if (imu_.wasReset()) reenableReports_();
      if (imu_.getSensorEvent(&ev_) && ev_.sensorId == SH2_GAME_ROTATION_VECTOR) {
        float r, p;
        rawRollPitchFromQuat_(ev_.un.gameRotationVector, r, p);
        sumR += r; sumP += p;
        n++;
      }
      delay(1);
    }
    if (n < 50) {
      Serial.printf("[IMU] ERROR: pocas muestras (%d), no calibra offset\n", n);
      return false;
    }
    rollOffsetDeg_  = (float)(sumR / n);
    pitchOffsetDeg_ = (float)(sumP / n);
    attOffRoll  = rollOffsetDeg_;
    attOffPitch = pitchOffsetDeg_;
    Serial.printf("[IMU] Offset mecánico R:%.2f° P:%.2f° (n=%d)\n",
                  rollOffsetDeg_, pitchOffsetDeg_, n);
    if (fabsf(rollOffsetDeg_) > 5.f || fabsf(pitchOffsetDeg_) > 5.f) {
      Serial.println("[IMU] ⚠ El dron NO está nivelado al calibrar.");
      return false;
    }
    return true;
  }

  void resetYaw() { yawDeg = 0.f; }

  // ── Bucle de lectura (llamar cada loop) ───────────────────────────────
  // budget: cuántos eventos del FIFO procesar antes de ceder
  void update(int budget = 12) {
    if (imu_.wasReset()) reenableReports_();
    bool got = false;
    for (int i = 0; i < budget; i++) {
      if (!imu_.getSensorEvent(&ev_)) break;
      got = true;
      totalSamples++;
      if (ev_.sensorId == SH2_GAME_ROTATION_VECTOR) {
        processQuat_(ev_.un.gameRotationVector);
      } else if (ev_.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
        processGyro_(ev_.un.gyroscopeUncal);
      }
    }
    if (got) lastEventMs = millis();
  }

  bool wasReset() { return imu_.wasReset(); }

private:
  Adafruit_BNO08x imu_{PIN_IMU_INT};
  sh2_SensorValue_t ev_;

  // Filtros
  flt::BiquadLPF bqRoll_, bqPitch_, bqGR_, bqGP_, bqGY_;
  flt::BiquadNotch notchGR_, notchGP_, notchGY_;
  flt::Median3     medGR_, medGP_, medGY_;
  flt::SpikeFilter spkRoll_, spkPitch_, spkGR_, spkGP_, spkGY_;

  // Estado interno
  float biasX_ = 0.f, biasY_ = 0.f, biasZ_ = 0.f;
  float rollOffsetDeg_ = 0.f, pitchOffsetDeg_ = 0.f;
  unsigned long lastGyroUs_ = 0;

  // ── Re-habilitar reportes tras un reset interno del IMU ───────────────
  void reenableReports_() {
    imu_.enableReport(SH2_GAME_ROTATION_VECTOR, 4000);
    imu_.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 4000);
  }

  // ── Cálculo del vector "up" en frame del dron ─────────────────────────
  //   q = (real, i, j, k) — convención BNO086
  //   Tercera fila de R(q) (sensor frame): up_sensor (mundo Z en sensor)
  //   Remap fijo: drone_x ← +sensor_y    drone_y ← -sensor_z    drone_z ← -sensor_x
  //   (consistente con V3.3 verificado en banco)
  //   roll  = atan2(-up_y_body, -up_z_body)
  //   pitch =  asin( up_x_body )
  static void rawRollPitchFromQuat_(const sh2_RotationVectorWAcc& /*qwa*/, float& /*r*/, float& /*p*/) {}
  static void rawRollPitchFromQuat_(const sh2_RotationVector_t& q,   float& rDeg, float& pDeg) {
    const float i = q.i, j = q.j, k = q.k, r = q.real;
    // up_sensor = R(q)·ẑ  (tercera columna de R aplicada a [0,0,1])
    const float ux_s = 2.0f * (i * k + j * r);
    const float uy_s = 2.0f * (j * k - i * r);
    const float uz_s = 1.0f - 2.0f * (i * i + j * j);
    // remap sensor → drone (standard flat mounting: X_b ← +X_s , Y_b ← +Y_s , Z_b ← -Z_s)
    const float ux_b =  ux_s;
    const float uy_b =  uy_s;
    const float uz_b = -uz_s;
    // ángulos por geometría
    pDeg = asinf(constrain(ux_b, -1.f, 1.f)) * 57.29577951f;
    rDeg = atan2f(-uy_b, -uz_b) * 57.29577951f;
  }

  void processQuat_(const sh2_RotationVector_t& q) {
    float r, p;
    rawRollPitchFromQuat_(q, r, p);
    r -= rollOffsetDeg_;
    p -= pitchOffsetDeg_;
    rollDeg  = bqRoll_.update (spkRoll_.update (r));
    pitchDeg = bqPitch_.update(spkPitch_.update(p));
    ready = true;
  }

  void processGyro_(const sh2_GyroscopeUncalibrated_t& g) {
    // bias removal
    float gx = g.x - biasX_;
    float gy = g.y - biasY_;
    float gz = g.z - biasZ_;
    // mapeo sensor → dron (standard flat: roll=X, pitch=Y, yaw=Z)
    constexpr float RAD2DEG = 57.29577951f;
    float rollRate  = GYRO_SIGN_ROLL  * gx * RAD2DEG;   // roll  ↔ sensor X
    float pitchRate = GYRO_SIGN_PITCH * gy * RAD2DEG;   // pitch ↔ sensor Y
    float yawRate   = GYRO_SIGN_YAW   * gz * RAD2DEG;   // yaw   ↔ sensor Z

    // cadena de filtros
    rollRate  = bqGR_.update(notchGR_.update(spkGR_.update(medGR_.update(rollRate))));
    pitchRate = bqGP_.update(notchGP_.update(spkGP_.update(medGP_.update(pitchRate))));
    yawRate   = bqGY_.update(notchGY_.update(spkGY_.update(medGY_.update(yawRate))));

    gyroRollDps  = rollRate;
    gyroPitchDps = pitchRate;
    gyroYawDps   = yawRate;

    // integración de yaw (estimación de heading sin magnetómetro)
    unsigned long nowUs = micros();
    if (lastGyroUs_ != 0) {
      float dt = constrain((float)(nowUs - lastGyroUs_) * 1e-6f, 0.0005f, 0.05f);
      yawDeg += yawRate * dt;
      while (yawDeg >  180.f) yawDeg -= 360.f;
      while (yawDeg < -180.f) yawDeg += 360.f;
    }
    lastGyroUs_ = nowUs;
  }
};
