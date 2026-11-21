// ============================================================================
//  Baro.h — BME688 wrapper NO BLOQUEANTE
//
//  IMPORTANTE: performReading() es BLOQUEANTE (~250-400 ms con OS_8X).
//  Si se llama desde el bucle de control, destroza la frecuencia del PID
//  (causa: dron no estabiliza). Aquí usamos la API asíncrona:
//      beginReading() → millis cuando estará lista
//      endReading()   → recoge la muestra cuando ya está disponible
//
//  Cadena de filtrado: raw → MovingAvg<N> → BiquadLPF(1.5 Hz) → LPF1
//  Velocidad vertical: derivada filtrada de la altitud final.
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME680.h>
#include "Config.h"
#include "Filters.h"

class Baro {
public:
  bool  ready       = false;
  float refPressure = 1013.25f;
  float pressureHpa = 0.f;
  float tempC       = 0.f;
  float altitudeM   = 0.f;
  float altitudeRawM= 0.f;

  bool begin() {
    if (!sensor_.begin(0x77) && !sensor_.begin(0x76)) {
      ready = false;
      return false;
    }
    // OS_2X presión ≈ 40 ms de conversión → cómoda para 20 Hz async
    sensor_.setTemperatureOversampling(BME680_OS_1X);
    sensor_.setPressureOversampling(BME680_OS_2X);
    sensor_.setHumidityOversampling(BME680_OS_NONE);
    sensor_.setIIRFilterSize(BME680_FILTER_SIZE_3);
    sensor_.setGasHeater(0, 0);

    biquad_.configure(ALT_BIQUAD_FC_HZ, BARO_FS_HZ);
    lpf_.setCutoff(ALT_BIQUAD_FC_HZ, BARO_FS_HZ);
    ready    = true;
    pending_ = false;
    return true;
  }

  // En setup() usamos lectura bloqueante (no hay control activo).
  bool captureReference(uint16_t samples = 30, uint32_t timeoutMs = 4000) {
    if (!ready) return false;
    uint32_t t0 = millis();
    uint16_t n  = 0;
    float    acc= 0.f;
    pending_ = false;
    while (n < samples && (millis() - t0) < timeoutMs) {
      if (sensor_.performReading()) {
        acc += sensor_.pressure / 100.0f;
        n++;
      }
      delay(40);
    }
    if (n == 0) return false;
    refPressure = acc / (float)n;
    ma_.reset(0.f);
    biquad_.reset(0.f);
    lpf_.reset(0.f);
    altitudeM    = 0.f;
    altitudeRawM = 0.f;
    prevAlt_     = 0.f;
    return true;
  }

  // NO bloqueante. Devuelve true SOLO cuando una nueva muestra se procesó.
  // Llamar muy frecuentemente desde el loop principal (no daña nada si llega
  // antes de que el sensor termine; simplemente retorna false).
  bool update() {
    if (!ready) return false;

    if (!pending_) {
      long rdy = sensor_.beginReading();
      if (rdy <= 0) return false;          // error al solicitar
      readyAtMs_ = (uint32_t)rdy;
      pending_   = true;
      return false;
    }
    if ((int32_t)(millis() - readyAtMs_) < 0) return false;   // aún no listo

    if (!sensor_.endReading()) {
      pending_ = false;
      return false;
    }
    pending_     = false;
    pressureHpa  = sensor_.pressure / 100.0f;
    tempC        = sensor_.temperature;
    altitudeRawM = 44330.0f * (1.0f - powf(pressureHpa / refPressure, 0.1903f));
    float mA = ma_.update(altitudeRawM);
    float bq = biquad_.update(mA);
    altitudeM = lpf_.update(bq);
    return true;
  }

  float verticalSpeed() const { return vSpeed_; }

  // Llamar tras update()==true con dt = (now − lastCompleted) en segundos.
  void updateVerticalSpeed(float dt) {
    if (dt < 1e-3f) return;
    float raw = (altitudeM - prevAlt_) / dt;
    prevAlt_  = altitudeM;
    vSpeed_   = vSpeedLpf_.update(raw);
  }

private:
  Adafruit_BME680 sensor_{&Wire};
  flt::MovingAvg<BARO_MA_SAMPLES> ma_;
  flt::BiquadLPF                  biquad_;
  flt::LPF1                       lpf_{ALPHA_ALT_LPF};
  flt::LPF1                       vSpeedLpf_{0.25f};
  float    prevAlt_   = 0.f;
  float    vSpeed_    = 0.f;
  bool     pending_   = false;
  uint32_t readyAtMs_ = 0;
};
