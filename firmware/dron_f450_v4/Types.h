// ============================================================================
//  Types.h — Estructuras compartidas drone↔remote (compatibles con V3.5)
// ============================================================================
#pragma once
#include <stdint.h>

struct __attribute__((packed)) DatosControl {
  int16_t  throttle;       // 1000..2000 µs
  int16_t  yaw;            // -500..+500
  int16_t  pitch;          // -500..+500
  int16_t  roll;           // -500..+500
  bool     armado;
  bool     emergencyStop;
  uint8_t  flightMode;     // 0=DISARM, 1=STABILIZE, 3=LOITER
  uint16_t modeSeq;
};

struct __attribute__((packed)) TelemetriaDron {
  bool imuListo;
  bool baroListo;
  bool dronArmado;
};

enum FlightMode : uint8_t {
  MODE_DISARMED  = 0,
  MODE_STABILIZE = 1,
  MODE_LOITER    = 3
};
