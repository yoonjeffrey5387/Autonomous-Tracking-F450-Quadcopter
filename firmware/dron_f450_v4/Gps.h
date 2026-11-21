// ============================================================================
//  Gps.h — TinyGPSPlus wrapper con quality gating
//    · Mínimo de satélites
//    · Máximo HDOP
//    · Edad máxima del fix (ms)
//    · Filtro biquad ligero sobre velocidad horizontal
// ============================================================================
#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "Config.h"
#include "Filters.h"

class Gps {
public:
  bool   hasFix       = false;
  bool   qualityOk    = false;
  double latitude     = 0.0;
  double longitude    = 0.0;
  float  altitudeM    = 0.f;
  float  speedMs      = 0.f;     // filtrada
  float  headingDeg   = 0.f;
  uint8_t satellites  = 0;
  float  hdop         = 99.9f;
  uint32_t fixAgeMs   = 0xFFFFFFFFUL;

  void begin(HardwareSerial& port, uint32_t baud = GPS_BAUD,
             int8_t rxPin = -1, int8_t txPin = -1) {
    port_ = &port;
    if (rxPin >= 0 || txPin >= 0) {
      port_->begin(baud, SERIAL_8N1, rxPin, txPin);
    } else {
      port_->begin(baud);
    }
    speedLpf_.setCutoff(0.8f, 10.f);
  }

  // A llamar a alta frecuencia (>=50 Hz) para no perder NMEA.
  void update() {
    if (!port_) return;
    while (port_->available()) {
      char c = (char)port_->read();
      gps_.encode(c);
    }
    if (gps_.location.isValid()) {
      latitude  = gps_.location.lat();
      longitude = gps_.location.lng();
      fixAgeMs  = gps_.location.age();
      hasFix    = true;
    }
    if (gps_.altitude.isValid()) altitudeM = (float)gps_.altitude.meters();
    if (gps_.speed.isValid())    speedMs   = speedLpf_.update((float)gps_.speed.mps());
    if (gps_.course.isValid())   headingDeg= (float)gps_.course.deg();
    if (gps_.satellites.isValid()) satellites = (uint8_t)gps_.satellites.value();
    if (gps_.hdop.isValid())     hdop      = (float)gps_.hdop.hdop();

    qualityOk = hasFix
             && satellites >= GPS_MIN_SATS
             && hdop       <= GPS_MAX_HDOP
             && fixAgeMs   <= GPS_MAX_AGE_MS;
  }

  uint32_t charsProcessed() const { return gps_.charsProcessed(); }

private:
  HardwareSerial* port_ = nullptr;
  TinyGPSPlus     gps_;
  flt::LPF1       speedLpf_{0.25f};
};
