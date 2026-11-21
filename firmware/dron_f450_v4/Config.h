// ============================================================================
//  Config.h — Constantes de compilación del Dron F450 (1 kg)
//  Hardware: ESP32-S3 / BNO086 / BME688 / GPS GP-02 / 4× 2212-920KV / 9450 / 3S
// ============================================================================
#pragma once

// ─── PINES ────────────────────────────────────────────────────────────────
#define PIN_M1 36   // Front-Right (CW)
#define PIN_M2 37   // Back-Left   (CW)
#define PIN_M3 38   // Front-Left  (CCW)
#define PIN_M4 39   // Back-Right  (CCW)
#define PIN_IMU_SDA 47
#define PIN_IMU_SCL 48
#define PIN_IMU_INT  5
#define PIN_GPS_RX  -1            // Serial1 default
#define PIN_GPS_TX  -1
#define GPS_BAUD  9600

// ─── ESC PWM ──────────────────────────────────────────────────────────────
#define ESC_PWM_FREQ_HZ   250
#define ESC_PWM_RES_BITS  12
#define ESC_MIN_US       1000
#define ESC_IDLE_US      1100      // armed idle (motores girando lento)
#define ESC_MAX_US       2000
#define ESC_SLEW_UP_US_S  900.0f   // rampa máxima de subida us/s
#define ESC_SLEW_DOWN_US_S 350.0f  // rampa máxima de bajada us/s

// ─── BUCLE PRINCIPAL ──────────────────────────────────────────────────────
#define LOOP_RATE_HZ          250
#define LOOP_DT_S             (1.0f / LOOP_RATE_HZ)
#define LOOP_PERIOD_US        (1000000UL / LOOP_RATE_HZ)
#define BARO_TRIGGER_US       50000UL   // 20 Hz
#define ALT_LOOP_US           50000UL
#define TELEMETRY_PERIOD_MS   200       // 5 Hz
#define SERIAL_STATUS_MS      500       // 2 Hz

// ─── ESP-NOW ──────────────────────────────────────────────────────────────
#define ESPNOW_CHANNEL  11
#define FAILSAFE_WARN_MS 300
#define FAILSAFE_CUT_MS  800

// ─── MOUNT TRANSFORM ──────────────────────────────────────────────────────
//  La IMU está orientada de modo no-estándar. Estos signos remapean los ejes
//  del sensor al frame del dron (X=adelante, Y=derecha, Z=abajo) y han sido
//  verificados en banco con V3.3. Cambia un signo si en diagnóstico Phase-4
//  el ángulo o tasa correspondiente sale invertido.
#define GYRO_SIGN_ROLL   (+1.0f)
#define GYRO_SIGN_PITCH  (-1.0f)
#define GYRO_SIGN_YAW    (-1.0f)   // invertido tras diagnóstico (CW = +)
// El remap angular ocurre en Attitude.h (rollAng ← sensor pitch, etc.).

// ─── FILTROS ──────────────────────────────────────────────────────────────
#define GYRO_BIQUAD_FC_HZ    80.0f
#define ANGLE_BIQUAD_FC_HZ   40.0f
#define ALT_BIQUAD_FC_HZ      1.5f
#define BARO_FS_HZ           20.0f
#define D_TERM_LPF_FC_HZ     65.0f      // LPF independiente del D-term
#define RC_LPF_FC_HZ         12.0f
#define ALPHA_THROTTLE        0.20f
#define ALPHA_ALT_LPF         0.15f
#define BARO_MA_SAMPLES       20
#define SPIKE_THR_ANGLE_DEG  30.0f
#define SPIKE_THR_GYRO_DPS  400.0f
// Notch opcional para vibración de motor (Hz). 0 = desactivado.
#define GYRO_NOTCH_FC_HZ      0.0f
#define GYRO_NOTCH_Q          6.0f

// ─── SENSIBILIDADES ───────────────────────────────────────────────────────
#define STICK_DEADZONE_RAW    20         // unidades de -500..+500
#define MAX_ANGLE_CMD_DEG     30.0f      // stick lleno → 30° de inclinación
#define MAX_ANGLE_LIMIT_DEG   45.0f      // saturación física
#define MAX_YAW_RATE_DPS     120.0f
#define MAX_LOITER_TILT_DEG   18.0f
#define MAX_RATE_FROM_ANGLE  220.0f      // tasa máxima emitida por bucle ángulo

// ─── PID — ÁNGULO (externo) ───────────────────────────────────────────────
#define ANGLE_KP_ROLL    5.0f
#define ANGLE_KP_PITCH   5.0f
#define ANGLE_KI_ROLL    0.05f
#define ANGLE_KI_PITCH   0.05f
#define ANGLE_KD         0.0f
#define ANGLE_ILIM       12.0f

// ─── PID — TASA (interno) ─────────────────────────────────────────────────
//  Punto de partida para 1 kg con 920 KV + 9450 + 3S. Afinar con T-menu.
#define RATE_KP_ROLL     0.55f
#define RATE_KI_ROLL     0.45f
#define RATE_KD_ROLL     0.022f
#define RATE_KFF_ROLL    0.035f
#define RATE_ILIM_ROLL  200.0f

#define RATE_KP_PITCH    0.50f
#define RATE_KI_PITCH    0.40f
#define RATE_KD_PITCH    0.020f
#define RATE_KFF_PITCH   0.035f
#define RATE_ILIM_PITCH 200.0f

#define YAW_KP           2.8f
#define YAW_KI           0.25f
#define YAW_KD           0.0f
#define YAW_KFF          0.0f
#define YAW_ILIM        30.0f

// ─── PID — ALTITUD Y POSICIÓN ─────────────────────────────────────────────
#define ALT_KP    2.0f
#define ALT_KI    0.05f
#define ALT_KD    5.0f
#define ALT_ILIM  200.0f

#define POS_KP    0.25f
#define POS_KI    0.004f
#define POS_KD    0.40f
#define POS_ILIM  8.0f

#define VEL_KP    0.40f
#define VEL_KI    0.003f
#define VEL_KD    0.12f
#define VEL_ILIM  8.0f
#define LOITER_VEL_DAMP 0.55f

// ─── THROTTLE / HOVER ─────────────────────────────────────────────────────
//  Para 1 kg con 4×2212-920KV + 9450 + 3S, el punto de hover típico es
//  alrededor de 1450-1520 µs. Valor de arranque conservador:
#define HOVER_THROTTLE_US   1500.0f

// TPA: Reduce ganancia P/D del bucle de tasa a throttle alto
#define TPA_BREAKPOINT_US   1500
#define TPA_ATTENUATION     0.35f

// ─── SEGURIDAD ────────────────────────────────────────────────────────────
#define ARM_THROTTLE_MAX     1080
#define ARM_MAX_TILT_DEG     12.0f
#define IMU_SETTLE_MS        4000
#define ARM_STABLE_MS         250
#define MODE_CHANGE_LOCK_MS   400
#define CRASH_ANGLE_DEG      75.0f
#define FLIP_ANGLE_DEG       60.0f
#define FLIP_WINDOW_MS       2000

#define IN_AIR_THROTTLE_US   1180
#define IN_AIR_HOLD_MS        700
#define GROUND_THROTTLE_THR_US 1080.0f

// ─── I-TERM RELAX / ANTI-GRAVITY ──────────────────────────────────────────
#define I_RELAX_SETPOINT_DPS  40.0f
#define ANTI_GRAVITY_GAIN     2.0f
#define ANTI_GRAVITY_THR_DELTA 25.0f

// ─── GPS QUALITY ──────────────────────────────────────────────────────────
#define GPS_MIN_SATS     6
#define GPS_MAX_HDOP     2.5f
#define GPS_MAX_AGE_MS   2000

// ─── ALT STICK ────────────────────────────────────────────────────────────
#define ALT_STICK_SCALE  1.0f
#define ALT_DEADBAND_US  60

// ─── DIAGNÓSTICO ──────────────────────────────────────────────────────────
#define DIAG_PROMPT       'D'           // tecla para entrar al wizard
#define DIAG_MOTOR_TEST_US 1150         // pulso de prueba (sin hélices)
