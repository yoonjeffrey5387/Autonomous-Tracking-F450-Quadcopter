// ============================================================================
//  Diagnostics.h — Asistente único de diagnóstico por serial
//
//  Una sola tecla ('D') lanza un asistente paso-a-paso que verifica:
//    1. Sample rate del IMU
//    2. Sesgo del giroscopio (drift estacionario)
//    3. Nivel mecánico (offset de actitud)
//    4. Respuesta angular por eje (verificar signos)
//    5. Rotación de yaw (verificar signo)
//    6. Ruido y deriva del barómetro
//    7. Fix y calidad del GPS
//    8. Enlace ESP-NOW con el control
//    9. Numeración de motores (uno por uno, SIN hélices)
//   10. Saneamiento de la mezcla
//   11. Reporte final
//
//  Cada paso espera confirmación por serial ('y' continuar, 'n' saltar,
//  'q' abortar). Mientras corre, los motores están desarmados salvo en
//  los pasos explícitos de motor — y siempre con prompt de seguridad.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Attitude.h"
#include "Baro.h"
#include "Gps.h"
#include "Mixer.h"
#include "RadioLink.h"

class Diagnostics {
public:
  Diagnostics(Attitude& a, Baro& b, Gps& g, Mixer& m, RadioLink& r)
    : att_(a), baro_(b), gps_(g), mix_(m), rad_(r) {}

  bool isActive() const { return active_; }

  // Si llega 'D' en serial, lanza el wizard (bloquea).
  void serviceTrigger() {
    if (!active_ && Serial.available()) {
      int c = Serial.peek();
      if (c == DIAG_PROMPT || c == 'd') {
        Serial.read();
        runWizard();
      }
    }
  }

private:
  Attitude&   att_;
  Baro&       baro_;
  Gps&        gps_;
  Mixer&      mix_;
  RadioLink&  rad_;
  bool        active_ = false;

  // ── utilidades de serial ─────────────────────────────────────────────
  void flushIn() { while (Serial.available()) Serial.read(); }
  char ask(const char* prompt) {
    Serial.print(prompt);
    Serial.print(F("  [y/n/q] > "));
    flushIn();
    while (true) {
      if (Serial.available()) {
        char c = (char)Serial.read();
        if (c == 'y' || c == 'Y') { Serial.println('y'); return 'y'; }
        if (c == 'n' || c == 'N') { Serial.println('n'); return 'n'; }
        if (c == 'q' || c == 'Q') { Serial.println('q'); return 'q'; }
      }
      delay(5);
    }
  }
  void header(const char* title, int step) {
    Serial.println();
    Serial.println(F("--------------------------------------------------------"));
    Serial.print(F("  PASO "));
    Serial.print(step);
    Serial.print(F(": "));
    Serial.println(title);
    Serial.println(F("--------------------------------------------------------"));
  }

  // ── wizard principal ─────────────────────────────────────────────────
  void runWizard() {
    active_ = true;
    mix_.disarm();
    Serial.println();
    Serial.println(F("========================================================"));
    Serial.println(F("   F450 V4 — ASISTENTE DE DIAGNÓSTICO PASO A PASO"));
    Serial.println(F("========================================================"));
    Serial.println(F(" SEGURIDAD: hélices RETIRADAS hasta el último paso."));
    Serial.println(F(" Responde con 'y' continuar, 'n' saltar, 'q' abortar."));

    bool ok = true;
    int s  = 1;
    ok &= step_imuRate(s++);     if (!active_) return;
    ok &= step_gyroBias(s++);    if (!active_) return;
    ok &= step_level(s++);       if (!active_) return;
    ok &= step_tiltRoll(s++);    if (!active_) return;
    ok &= step_tiltPitch(s++);   if (!active_) return;
    ok &= step_yaw(s++);         if (!active_) return;
    ok &= step_baro(s++);        if (!active_) return;
    ok &= step_gps(s++);         if (!active_) return;
    ok &= step_radio(s++);       if (!active_) return;
    ok &= step_motorMap(s++);    if (!active_) return;
    ok &= step_mixSanity(s++);   if (!active_) return;
    report(ok);
    active_ = false;
  }

  // ── PASO 1: tasa del IMU ─────────────────────────────────────────────
  bool step_imuRate(int s) {
    header("Tasa de muestreo IMU", s);
    Serial.println(F(" Mido durante 3 segundos cuántas muestras emite el BNO086."));
    if (ask(" Continuar?") != 'y') return false;
    uint32_t t0 = millis();
    uint32_t startSamples = att_.totalSamples;
    while (millis() - t0 < 3000) { att_.update(8); delay(1); }
    uint32_t got = att_.totalSamples - startSamples;
    float hz = got / 3.0f;
    Serial.print(F("  Muestras: ")); Serial.print(got);
    Serial.print(F("   ≈ "));        Serial.print(hz, 1); Serial.println(F(" Hz"));
    bool good = hz > 150.f;
    Serial.println(good ? F("  ✅ OK")
                        : F("  ⚠  BAJO. Revisa I2C / pull-ups / longitud cables."));
    return good;
  }

  // ── PASO 2: bias del giroscopio ──────────────────────────────────────
  bool step_gyroBias(int s) {
    header("Sesgo (bias) del giroscopio", s);
    Serial.println(F(" Coloca el DRON ABSOLUTAMENTE QUIETO sobre una superficie estable."));
    if (ask(" Continuar?") != 'y') return false;
    Serial.println(F("  Capturando 2 s de muestras..."));
    bool ok = att_.calibrateGyroBias(500, 2500);
    Serial.print(F("  Bias dps -> R:")); Serial.print(att_.gyroBiasRoll, 3);
    Serial.print(F("  P:"));             Serial.print(att_.gyroBiasPitch, 3);
    Serial.print(F("  Y:"));             Serial.println(att_.gyroBiasYaw, 3);
    Serial.println(ok ? F("  ✅ OK") : F("  ✘ FALLÓ — IMU no entrega datos."));
    return ok;
  }

  // ── PASO 3: nivel mecánico ───────────────────────────────────────────
  bool step_level(int s) {
    header("Nivel mecánico (offset de actitud)", s);
    Serial.println(F(" Coloca el dron PERFECTAMENTE NIVELADO (usa nivel de burbuja)."));
    if (ask(" Continuar?") != 'y') return false;
    bool ok = att_.calibrateAttitudeOffset(400, 2500);
    Serial.print(F("  Offset (deg)  R:")); Serial.print(att_.attOffRoll, 2);
    Serial.print(F("  P:"));                Serial.println(att_.attOffPitch, 2);
    Serial.println(ok ? F("  ✅ OK") : F("  ✘ FALLÓ"));
    return ok;
  }

  // ── PASO 4a: respuesta de ROLL ───────────────────────────────────────
  bool step_tiltRoll(int s) {
    header("Verificar signo de ROLL (eje X)", s);
    Serial.println(F(" Inclina el dron INCLINANDO EL LADO DERECHO HACIA ABAJO,"));
    Serial.println(F(" mantén ~30°, luego suelta. (Mantén pitch ≈ 0.)"));
    Serial.println(F(" Capturo durante 8 s — buscaré el pico de roll."));
    if (ask(" Continuar?") != 'y') return false;
    uint32_t t0 = millis();
    float peakRpos = 0.f, peakRneg = 0.f, peakPitchAbs = 0.f;
    while (millis() - t0 < 8000) {
      att_.update(6);
      if (att_.rollDeg > peakRpos) peakRpos = att_.rollDeg;
      if (att_.rollDeg < peakRneg) peakRneg = att_.rollDeg;
      if (fabsf(att_.pitchDeg) > peakPitchAbs) peakPitchAbs = fabsf(att_.pitchDeg);
      if ((millis() - t0) % 200 < 20) {
        Serial.print(F("  R:")); Serial.print(att_.rollDeg, 1);
        Serial.print(F("  P:")); Serial.println(att_.pitchDeg, 1);
        delay(20);
      }
    }
    float peakMag = fmaxf(peakRpos, -peakRneg);
    bool signPositive = (peakRpos >= -peakRneg);
    Serial.print(F("  Pico Roll: +")); Serial.print(peakRpos, 1);
    Serial.print(F("° / "));           Serial.print(peakRneg, 1);
    Serial.print(F("°    sangrado pitch (debería ≈0): ")); Serial.print(peakPitchAbs, 1); Serial.println("°");
    if (peakMag < 15.f) {
      Serial.println(F("  ⚠ Pico muy pequeño — no se inclinó lo suficiente."));
      return false;
    }
    if (signPositive) {
      Serial.println(F("  ✅ ROLL OK (right-down → +)"));
      return true;
    }
    Serial.println(F("  ✘ ROLL INVERTIDO — hay que negar roll en el mount transform"));
    Serial.println(F("     (Attitude.h: cambiar 'rDeg = atan2f(-uy_b, -uz_b)' por '+uy_b')."));
    return false;
  }

  // ── PASO 4b: respuesta de PITCH ──────────────────────────────────────
  bool step_tiltPitch(int s) {
    header("Verificar signo de PITCH (eje Y)", s);
    Serial.println(F(" Levanta la NARIZ DEL DRON ~30°, luego baja. (Mantén roll ≈ 0.)"));
    Serial.println(F(" Capturo durante 8 s — buscaré el pico de pitch."));
    if (ask(" Continuar?") != 'y') return false;
    uint32_t t0 = millis();
    float peakPpos = 0.f, peakPneg = 0.f, peakRollAbs = 0.f;
    while (millis() - t0 < 8000) {
      att_.update(6);
      if (att_.pitchDeg > peakPpos) peakPpos = att_.pitchDeg;
      if (att_.pitchDeg < peakPneg) peakPneg = att_.pitchDeg;
      if (fabsf(att_.rollDeg) > peakRollAbs) peakRollAbs = fabsf(att_.rollDeg);
      if ((millis() - t0) % 200 < 20) {
        Serial.print(F("  R:")); Serial.print(att_.rollDeg, 1);
        Serial.print(F("  P:")); Serial.println(att_.pitchDeg, 1);
        delay(20);
      }
    }
    float peakMag = fmaxf(peakPpos, -peakPneg);
    bool signPositive = (peakPpos >= -peakPneg);
    Serial.print(F("  Pico Pitch: +")); Serial.print(peakPpos, 1);
    Serial.print(F("° / "));            Serial.print(peakPneg, 1);
    Serial.print(F("°    sangrado roll (debería ≈0): ")); Serial.print(peakRollAbs, 1); Serial.println("°");
    if (peakMag < 15.f) {
      Serial.println(F("  ⚠ Pico muy pequeño — no se inclinó lo suficiente."));
      return false;
    }
    if (signPositive) {
      Serial.println(F("  ✅ PITCH OK (nose-up → +)"));
      return true;
    }
    Serial.println(F("  ✘ PITCH INVERTIDO — negar pDeg en mount transform"));
    Serial.println(F("     (Attitude.h: cambiar 'pDeg = asinf(ux_b)' por '-asinf(ux_b)')."));
    return false;
  }

  // ── PASO 5: rotación de yaw ──────────────────────────────────────────
  bool step_yaw(int s) {
    header("Verificar signo de YAW", s);
    Serial.println(F(" Resetea yaw a 0 y luego rota el dron en sentido HORARIO visto"));
    Serial.println(F(" desde arriba → yaw debe AUMENTAR (+)."));
    if (ask(" Continuar?") != 'y') return false;
    att_.resetYaw();
    uint32_t t0 = millis();
    while (millis() - t0 < 8000) {
      att_.update(6);
      if ((millis() - t0) % 250 < 20) {
        Serial.print(F("  yaw: ")); Serial.print(att_.yawDeg, 1);
        Serial.println(F(" deg"));
        delay(20);
      }
    }
    char a = ask(" ¿Yaw aumentó en sentido horario?");
    if (a == 'n') Serial.println(F(" >> Invierte GYRO_SIGN_YAW en Config.h."));
    return a == 'y';
  }

  // ── PASO 6: ruido y deriva del barómetro ─────────────────────────────
  bool step_baro(int s) {
    header("Ruido y deriva del barómetro", s);
    if (!baro_.ready) {
      Serial.println(F("  ✘ BME688 no inicializado — salto este paso."));
      return false;
    }
    Serial.println(F(" Mido 5 s con el dron quieto. Std-dev debe ser < 0.3 m."));
    if (ask(" Continuar?") != 'y') return false;
    uint32_t t0 = millis();
    int n = 0;
    float sum = 0.f, sumSq = 0.f;
    while (millis() - t0 < 5000) {
      if (baro_.update()) {
        sum   += baro_.altitudeM;
        sumSq += baro_.altitudeM * baro_.altitudeM;
        n++;
      }
      delay(45);
    }
    if (n < 5) { Serial.println(F("  ✘ Pocas muestras")); return false; }
    float mean = sum / n;
    float var  = (sumSq / n) - mean * mean;
    float sd   = sqrtf(var > 0 ? var : 0);
    Serial.print(F("  muestras=")); Serial.print(n);
    Serial.print(F("  media="));    Serial.print(mean, 2);
    Serial.print(F(" m  σ="));      Serial.println(sd, 3);
    bool ok = sd < 0.30f;
    Serial.println(ok ? F("  ✅ OK") : F("  ⚠  Ruido alto — protege la toma del barómetro de la corriente de aire."));
    return ok;
  }

  // ── PASO 7: GPS ──────────────────────────────────────────────────────
  bool step_gps(int s) {
    header("Calidad del GPS", s);
    Serial.println(F(" Verificando fix. (Saltable si vuelas indoor)."));
    if (ask(" Continuar?") != 'y') return false;
    uint32_t t0 = millis();
    while (millis() - t0 < 5000) { gps_.update(); delay(50); }
    Serial.print(F("  sats=")); Serial.print(gps_.satellites);
    Serial.print(F("  hdop=")); Serial.print(gps_.hdop, 2);
    Serial.print(F("  age="));  Serial.print(gps_.fixAgeMs);
    Serial.print(F("  fix=")); Serial.println(gps_.hasFix ? "Y" : "N");
    bool ok = gps_.qualityOk;
    Serial.println(ok ? F("  ✅ OK") : F("  ⚠  GPS sin fix válido — modo LOITER deshabilitado."));
    return ok;
  }

  // ── PASO 8: enlace ESP-NOW ───────────────────────────────────────────
  bool step_radio(int s) {
    header("Enlace ESP-NOW (control remoto)", s);
    Serial.println(F(" Enciende el control y muévelo. Verifico recepción 4 s."));
    if (ask(" Continuar?") != 'y') return false;
    uint32_t startPkts = rad_.packetsReceived;
    delay(4000);
    uint32_t got = rad_.packetsReceived - startPkts;
    Serial.print(F("  paquetes recibidos: ")); Serial.println(got);
    Serial.print(F("  peer aprendido: ")); Serial.println(rad_.peerLearned ? "SI" : "NO");
    auto rc = rad_.snapshot();
    Serial.print(F("  THR="));   Serial.print(rc.throttle);
    Serial.print(F("  YAW="));   Serial.print(rc.yaw);
    Serial.print(F("  PIT="));   Serial.print(rc.pitch);
    Serial.print(F("  ROL="));   Serial.print(rc.roll);
    Serial.print(F("  arm="));   Serial.print(rc.armado);
    Serial.print(F("  mode=")); Serial.println(rc.flightMode);
    bool ok = got > 20;
    Serial.println(ok ? F("  ✅ OK") : F("  ⚠  Sin tráfico suficiente."));
    return ok;
  }

  // ── PASO 9: numeración de motores (sin hélices) ──────────────────────
  bool step_motorMap(int s) {
    header("Numeración de motores  (¡SIN HÉLICES!)", s);
    Serial.println(F(" CONFIRMACIÓN: las hélices están RETIRADAS, ¿sí?"));
    if (ask("  Confirmar 'y' SOLO si NO hay hélices puestas") != 'y') return false;
    const char* names[4] = {
      "M1  Front-Right  (CW)",
      "M2  Back-Left    (CW)",
      "M3  Front-Left   (CCW)",
      "M4  Back-Right   (CCW)"
    };
    for (int i = 0; i < 4; i++) {
      Serial.print(F("\n Probando ")); Serial.println(names[i]);
      Serial.println(F("  Verás girar UN motor 2 s. Pulsa 'y' para arrancar."));
      if (ask("  Listo?") != 'y') { mix_.disarm(); return false; }
      mix_.testMotor(i, DIAG_MOTOR_TEST_US);
      delay(2000);
      mix_.disarm();
      char a = ask("  ¿Giró el motor CORRECTO?");
      if (a != 'y') {
        Serial.println(F("  >> Reasigna pines o cables de motor."));
        return false;
      }
    }
    return true;
  }

  // ── PASO 10: saneamiento de la mezcla ───────────────────────────────
  bool step_mixSanity(int s) {
    header("Saneamiento de la mezcla X-quad", s);
    Serial.println(F(" Simulo entradas PID y muestro pulsos esperados."));
    Serial.println(F(" Sin hélices, los motores NO giran (mixer está aislado)."));
    if (ask(" Continuar?") != 'y') return false;
    struct C { const char* name; float t,r,p,y; };
    C cases[] = {
      { "ROLL +200 (derecha abajo)  → M2,M3 deben SUBIR",   1500,  200,   0,    0 },
      { "PITCH+200 (nariz arriba)   → M2,M4 deben SUBIR",   1500,   0,   200,   0 },
      { "YAW  +200 (giro CW)        → M3,M4 deben SUBIR",   1500,   0,   0,   200 },
    };
    for (auto& c : cases) {
      // simulación pura (no se mueven motores; usamos cálculo offline)
      float pidR = c.r, pidP = c.p, pidY = c.y, t = c.t;
      float m1 = t - pidP - pidR - pidY;
      float m2 = t + pidP + pidR - pidY;
      float m3 = t - pidP + pidR + pidY;
      float m4 = t + pidP - pidR + pidY;
      Serial.print(F("  "));
      Serial.println(c.name);
      Serial.print(F("    M1=")); Serial.print((int)m1);
      Serial.print(F("  M2=")); Serial.print((int)m2);
      Serial.print(F("  M3=")); Serial.print((int)m3);
      Serial.print(F("  M4=")); Serial.println((int)m4);
    }
    return true;
  }

  void report(bool ok) {
    Serial.println();
    Serial.println(F("========================================================"));
    Serial.println(ok ? F(" RESULTADO: TODOS LOS PASOS PASARON ✅")
                      : F(" RESULTADO: ALGÚN PASO FALLÓ ⚠  — revisa el log."));
    Serial.println(F(" Cuando todo esté verde, monta hélices y arma normalmente."));
    Serial.println(F("========================================================"));
  }
};
