// ============================================================================
//  Filters.h — Primitivas de filtrado (header-only)
//  Diseñadas para 250 Hz (gyro/ángulo) y 20 Hz (baro). Encapsuladas en
//  flt:: namespace para evitar colisiones de nombres.
// ============================================================================
#pragma once
#include <Arduino.h>
#include <math.h>

namespace flt {

// ─── LPF 1er orden (suavizado simple, baja latencia) ─────────────────────
class LPF1 {
  float y_ = 0.f, a_ = 0.15f;
public:
  LPF1() = default;
  explicit LPF1(float alpha) : a_(alpha) {}
  void setAlpha(float a) { a_ = a; }
  void setCutoff(float fc_hz, float fs_hz) {
    if (fc_hz <= 0.f) { a_ = 1.f; return; }
    float rc = 1.0f / (2.0f * (float)M_PI * fc_hz);
    float dt = 1.0f / fs_hz;
    a_ = dt / (rc + dt);
  }
  float update(float x) { y_ += a_ * (x - y_); return y_; }
  void  reset(float v)  { y_ = v; }
  float value() const   { return y_; }
};

// ─── Biquad Butterworth LPF (2º orden, DF-II transpuesta) ───────────────
class BiquadLPF {
  float b0_=1, b1_=0, b2_=0, a1_=0, a2_=0;
  float s1_=0, s2_=0;
public:
  BiquadLPF() = default;
  BiquadLPF(float fc_hz, float fs_hz) { configure(fc_hz, fs_hz); }
  void configure(float fc_hz, float fs_hz) {
    if (fc_hz <= 0.f || fc_hz >= fs_hz * 0.49f) {
      b0_=1; b1_=0; b2_=0; a1_=0; a2_=0;       // pasa-todo
      return;
    }
    float K = tanf((float)M_PI * fc_hz / fs_hz);
    float norm = 1.0f / (1.0f + (float)M_SQRT2 * K + K * K);
    b0_ = K * K * norm;
    b1_ = 2.0f * b0_;
    b2_ = b0_;
    a1_ = 2.0f * (K * K - 1.0f) * norm;
    a2_ = (1.0f - (float)M_SQRT2 * K + K * K) * norm;
  }
  float update(float x) {
    float y = b0_ * x + s1_;
    s1_ = b1_ * x - a1_ * y + s2_;
    s2_ = b2_ * x - a2_ * y;
    return y;
  }
  void reset(float v) {
    // estados estacionarios para entrada constante v
    s1_ = v * (b1_ + b2_ - a1_ - a2_);
    s2_ = v * (b2_ - a2_);
  }
};

// ─── Notch (rechaza-banda) — vibración de motor ─────────────────────────
class BiquadNotch {
  float b0_=1, b1_=0, b2_=0, a1_=0, a2_=0;
  float s1_=0, s2_=0;
public:
  void configure(float fc_hz, float fs_hz, float Q) {
    if (fc_hz <= 0.f || fc_hz >= fs_hz * 0.49f || Q <= 0.f) {
      b0_=1; b1_=0; b2_=0; a1_=0; a2_=0;
      return;
    }
    float w0 = 2.0f * (float)M_PI * fc_hz / fs_hz;
    float cw = cosf(w0);
    float alpha = sinf(w0) / (2.0f * Q);
    float a0 = 1.0f + alpha;
    b0_ =  1.0f / a0;
    b1_ = -2.0f * cw / a0;
    b2_ =  b0_;
    a1_ =  b1_;
    a2_ = (1.0f - alpha) / a0;
  }
  float update(float x) {
    float y = b0_ * x + s1_;
    s1_ = b1_ * x - a1_ * y + s2_;
    s2_ = b2_ * x - a2_ * y;
    return y;
  }
  void reset(float /*v*/) { s1_ = 0; s2_ = 0; }
};

// ─── Mediana de 3 (rechaza spikes aislados con latencia ≤1 muestra) ─────
class Median3 {
  float a_=0, b_=0;
public:
  float update(float c) {
    float m;
    if      ((a_ <= b_ && b_ <= c) || (c <= b_ && b_ <= a_)) m = b_;
    else if ((b_ <= a_ && a_ <= c) || (c <= a_ && a_ <= b_)) m = a_;
    else                                                      m = c;
    a_ = b_; b_ = c;
    return m;
  }
  void reset(float v) { a_ = b_ = v; }
};

// ─── Spike filter (rechaza muestras con salto > umbral) ─────────────────
class SpikeFilter {
  float prev_ = 0.f, thr_;
  int   reject_ = 0;
public:
  explicit SpikeFilter(float threshold = 20.f) : thr_(threshold) {}
  float update(float x) {
    if (fabsf(x - prev_) > thr_ && reject_ < 3) {
      reject_++;
      return prev_;
    }
    reject_ = 0;
    prev_ = x;
    return x;
  }
  void reset(float v) { prev_ = v; reject_ = 0; }
};

// ─── Promedio móvil (N muestras) ────────────────────────────────────────
template<int N>
class MovingAvg {
  float buf_[N] = {};
  int   idx_ = 0, cnt_ = 0;
  float sum_ = 0.f;
public:
  float update(float v) {
    sum_ -= buf_[idx_];
    buf_[idx_] = v;
    sum_ += v;
    idx_ = (idx_ + 1) % N;
    if (cnt_ < N) cnt_++;
    return sum_ / (float)cnt_;
  }
  void reset(float v) {
    for (int i = 0; i < N; i++) buf_[i] = v;
    sum_ = v * N; cnt_ = N; idx_ = 0;
  }
};

// ─── Slew-rate limiter (limita d/dt) ────────────────────────────────────
class SlewLimiter {
  float prev_ = 0.f;
  float upPerSec_, downPerSec_;
public:
  SlewLimiter(float upPerSec = 1000.f, float downPerSec = 1000.f)
    : upPerSec_(upPerSec), downPerSec_(downPerSec) {}
  void setRates(float up, float down) { upPerSec_ = up; downPerSec_ = down; }
  float update(float target, float dt) {
    float maxUp   =  upPerSec_   * dt;
    float maxDown = -downPerSec_ * dt;
    float delta = constrain(target - prev_, maxDown, maxUp);
    prev_ += delta;
    return prev_;
  }
  void  reset(float v) { prev_ = v; }
  float value() const  { return prev_; }
};

} // namespace flt
