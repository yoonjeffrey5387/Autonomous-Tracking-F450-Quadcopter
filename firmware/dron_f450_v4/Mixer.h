// ============================================================================
//  Mixer.h — Mezclador X-quad profesional con:
//    · TPA (Throttle PID Attenuation) — atenúa P/D a throttle alto
//    · Slew limiter por motor (rampa físicamente realista para el ESC)
//    · Idle floor: 1100 µs cuando armado, 1000 cuando desarmado
//    · Detección de saturación real del actuador (en límites físicos, no slew)
//    · Salida en µs ya escrita a los 4 canales LEDC
//
//  Convención (X-quad mirando desde arriba, nariz arriba):
//      M3 (FL CCW)    M1 (FR CW)
//                X
//      M2 (BL CW)     M4 (BR CCW)
//
//   M1 = T - P - R - Y      (FR CW)
//   M2 = T + P + R - Y      (BL CW)
//   M3 = T - P + R + Y      (FL CCW)
//   M4 = T + P - R + Y      (BR CCW)
// ============================================================================
#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Filters.h"

class Mixer {
public:
  bool   motorsSaturatedHigh = false;
  bool   motorsSaturatedLow  = false;
  int    pulseUs[4]          = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};

  void begin() {
    ledcAttach(PIN_M1, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
    ledcAttach(PIN_M2, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
    ledcAttach(PIN_M3, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
    ledcAttach(PIN_M4, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
    for (int i = 0; i < 4; i++) {
      slew_[i].setRates(ESC_SLEW_UP_US_S, ESC_SLEW_DOWN_US_S);
      slew_[i].reset((float)ESC_MIN_US);
      pulseUs[i] = ESC_MIN_US;
      writeRaw(i, ESC_MIN_US);
    }
  }

  // Apaga todos los motores (señal mínima ESC_MIN_US).
  void disarm() {
    for (int i = 0; i < 4; i++) {
      slew_[i].reset((float)ESC_MIN_US);
      pulseUs[i] = ESC_MIN_US;
      writeRaw(i, ESC_MIN_US);
    }
    motorsSaturatedHigh = false;
    motorsSaturatedLow  = false;
  }

  // Calcula el factor TPA [0..1] a partir del throttle bruto.
  static float tpaScale(float throttleUs) {
    if (throttleUs <= (float)TPA_BREAKPOINT_US) return 1.0f;
    float frac = (throttleUs - (float)TPA_BREAKPOINT_US)
               / (float)(ESC_MAX_US - TPA_BREAKPOINT_US);
    frac = constrain(frac, 0.0f, 1.0f);
    return 1.0f - TPA_ATTENUATION * frac;
  }

  // throttleUs : 1000..2000  (incluye hover + corrección de altura)
  // pidR/P/Y   : salida del bucle de rate (ya escalada en µs equivalentes)
  // armed      : si false, fuerza ESC_MIN_US sin slew
  void mix(float throttleUs, float pidR, float pidP, float pidY, bool armed, float dt) {
    if (!armed) {
      disarm();
      return;
    }

    // Aplica TPA al P+D rate (entra ya combinado en pidR/P/Y).
    // Para mantener autoridad de yaw lo dejamos sin TPA.
    float tpa = tpaScale(throttleUs);
    pidR *= tpa;
    pidP *= tpa;

    float t = constrain(throttleUs, (float)ESC_IDLE_US, (float)ESC_MAX_US);

    float m1 = t - pidP - pidR - pidY;   // FR CW
    float m2 = t + pidP + pidR - pidY;   // BL CW
    float m3 = t - pidP + pidR + pidY;   // FL CCW
    float m4 = t + pidP - pidR + pidY;   // BR CCW

    float raw[4] = { m1, m2, m3, m4 };

    // ── Reparto de saturación: si algún canal excede 2000, restamos el
    //    exceso por igual a todos para preservar la mezcla diferencial ──
    float maxV = raw[0];
    for (int i = 1; i < 4; i++) if (raw[i] > maxV) maxV = raw[i];
    if (maxV > (float)ESC_MAX_US) {
      float excess = maxV - (float)ESC_MAX_US;
      for (int i = 0; i < 4; i++) raw[i] -= excess;
    }
    float minV = raw[0];
    for (int i = 1; i < 4; i++) if (raw[i] < minV) minV = raw[i];
    if (minV < (float)ESC_IDLE_US) {
      float lack = (float)ESC_IDLE_US - minV;
      for (int i = 0; i < 4; i++) raw[i] += lack;
    }

    motorsSaturatedHigh = false;
    motorsSaturatedLow  = false;
    for (int i = 0; i < 4; i++) {
      float clamped = constrain(raw[i], (float)ESC_IDLE_US, (float)ESC_MAX_US);
      if (clamped >= (float)ESC_MAX_US - 1.0f) motorsSaturatedHigh = true;
      if (clamped <= (float)ESC_IDLE_US + 1.0f) motorsSaturatedLow  = true;

      float lim = slew_[i].update(clamped, dt);
      int us = (int)lroundf(lim);
      us = constrain(us, ESC_IDLE_US, ESC_MAX_US);
      pulseUs[i] = us;
      writeRaw(i, us);
    }
  }

  // Test mode: envía un pulso fijo a un único motor (sin hélices!).
  void testMotor(int idx, int us) {
    if (idx < 0 || idx >= 4) return;
    us = constrain(us, ESC_MIN_US, ESC_MAX_US);
    for (int i = 0; i < 4; i++) {
      int v = (i == idx) ? us : ESC_MIN_US;
      slew_[i].reset((float)v);
      pulseUs[i] = v;
      writeRaw(i, v);
    }
  }

private:
  flt::SlewLimiter slew_[4];
  const uint8_t motorPins_[4] = { PIN_M1, PIN_M2, PIN_M3, PIN_M4 };

  void writeRaw(int ch, int us) {
    // 12-bit @ 250 Hz: 1 µs = 1.024 cuentas. Periodo 4000 µs → 4096 cuentas.
    uint32_t maxDuty = (1UL << ESC_PWM_RES_BITS) - 1UL;
    uint32_t periodUs = 1000000UL / ESC_PWM_FREQ_HZ;
    uint32_t duty = ((uint32_t)us * maxDuty) / periodUs;
    if (duty > maxDuty) duty = maxDuty;
    ledcWrite(motorPins_[ch], duty);
  }
};
