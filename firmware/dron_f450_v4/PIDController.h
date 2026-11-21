// ============================================================================
//  PIDController.h — PID profesional con:
//    · D-term sobre la medición (no sobre el error) → sin "derivative kick"
//    · LPF biquad dedicado al término D
//    · Anti-windup condicional (sólo bloquea si el motor está saturado y la
//      integración seguiría empeorando la saturación)
//    · Bloqueo de integración cuando el dron está en suelo
//    · "I-term relax": reduce ki cuando el setpoint cambia rápido
//    · Feed-forward proporcional a la tasa del setpoint (kff)
//    · Boost temporal opcional del integrador (anti-gravity)
// ============================================================================
#pragma once
#include <Arduino.h>
#include "Filters.h"

class PIDController {
public:
  float kp = 0.f, ki = 0.f, kd = 0.f, kff = 0.f;
  float iLimit = 200.f;
  float outLimit = 1e6f;
  bool  iRelaxEnabled = true;
  float iRelaxThresholdDps = 40.f;   // por encima de esta tasa de setpoint, ki escala a 0

  PIDController() = default;
  PIDController(float p, float i, float d, float ff, float ilim) {
    setGains(p, i, d, ff, ilim);
  }
  void setGains(float p, float i, float d, float ff, float ilim) {
    kp = p; ki = i; kd = d; kff = ff; iLimit = ilim;
  }
  void setDFilter(float fc_hz, float fs_hz) { dFilter_.configure(fc_hz, fs_hz); }
  void setIRelax(bool enabled, float thresholdDps = 40.f) {
    iRelaxEnabled = enabled;
    iRelaxThresholdDps = thresholdDps;
  }

  // Cálculo principal. Devuelve la salida del PID.
  //   antiWindupActive: true si el actuador (motor) está en su límite físico
  //   groundLockout   : true para bloquear integración (dron en suelo)
  //   iBoost          : multiplicador temporal del integrador (anti-gravity, etc.)
  float update(float setpoint, float measurement, float dt,
               bool antiWindupActive = false,
               bool groundLockout    = false,
               float iBoost          = 1.0f)
  {
    if (dt < 1e-5f) dt = 1e-5f;
    if (!initialized_) {
      prevMeasurement_ = measurement;
      prevSetpoint_    = setpoint;
      initialized_     = true;
    }

    const float err = setpoint - measurement;

    // ── Feed-forward (tasa del setpoint) ────────────────────────────────
    const float spRate = (setpoint - prevSetpoint_) / dt;
    const float ffTerm = kff * spRate;

    // ── Proporcional ────────────────────────────────────────────────────
    const float pTerm = kp * err;

    // ── Integrador con conditional integration + I-relax ────────────────
    float iScale = 1.0f;
    if (iRelaxEnabled && iRelaxThresholdDps > 0.f) {
      float r = fabsf(spRate) / iRelaxThresholdDps;
      iScale = 1.0f - constrain(r, 0.0f, 1.0f);
    }
    const bool blockI = groundLockout ||
                        (antiWindupActive && (err * integ_) > 0.0f);
    if (!blockI) {
      integ_ += err * dt * iScale * iBoost;
      integ_ = constrain(integ_, -iLimit, iLimit);
    }
    const float iTerm = ki * integ_;

    // ── Derivativo SOBRE LA MEDICIÓN (filtrado) ─────────────────────────
    // d(err)/dt = d(sp)/dt - d(meas)/dt ; al usar sólo -d(meas)/dt evitamos
    // el "kick" cuando el setpoint salta. Para inputs determinísticos se
    // recupera el equivalente sumando el feed-forward.
    const float dInputRaw = -(measurement - prevMeasurement_) / dt;
    const float dFiltered = dFilter_.update(dInputRaw);
    const float dTerm = kd * dFiltered;

    prevMeasurement_ = measurement;
    prevSetpoint_    = setpoint;

    float out = pTerm + iTerm + dTerm + ffTerm;
    return constrain(out, -outLimit, outLimit);
  }

  void reset() {
    integ_ = 0.f;
    prevMeasurement_ = 0.f;
    prevSetpoint_ = 0.f;
    dFilter_.reset(0.f);
    initialized_ = false;
  }
  float integral()  const { return integ_; }
  void  setIntegral(float v) { integ_ = constrain(v, -iLimit, iLimit); }

private:
  float integ_ = 0.f;
  float prevMeasurement_ = 0.f;
  float prevSetpoint_    = 0.f;
  bool  initialized_     = false;
  flt::BiquadLPF dFilter_;
};
