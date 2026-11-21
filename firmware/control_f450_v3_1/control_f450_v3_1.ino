// ================================================================
//  CONTROL DRON F450 — FIRMWARE V3.5
//  CAMBIOS vs V3.4:
//   - Canal 11 (menos congestionado)
//   - Protocolo WiFi estándar (NO Long-Range) — menor latencia
//   - Sincronizado con dron V4.2
//   - BTN_ARM: hold 180ms-2999ms = arm/disarm
//                hold >= 3000ms   = cambiar modo STABILIZE<->LOITER
//   - LED_MODE parpadea durante hold largo para indicar que ya puede soltar
// ================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ─── MAC DEL DRON ────────────────────────────────────────────────
uint8_t macDron[] = {0x24, 0x58, 0x7C, 0xF5, 0x6C, 0xE4};

// ─── PINES ───────────────────────────────────────────────────────
#define JOY1_X     36   // Yaw
#define JOY1_Y     39   // Throttle
#define JOY2_X     34   // Roll
#define JOY2_Y     35   // Pitch
#define BTN_ARM    27
#define BTN_ESTOP  12
#define LED_ARM    32
#define LED_MODE   33
#define LED_CONN   25
#define BUZZER     26
#define ESPNOW_CHANNEL  11   // Canal 11 — menos congestionado (debe coincidir con el dron)

struct DatosControl {
  int16_t  throttle;
  int16_t  yaw;
  int16_t  pitch;
  int16_t  roll;
  bool     armado;
  bool     emergencyStop;
  uint8_t  flightMode;
  uint16_t modeSeq;
};
DatosControl txDatos;

struct TelemetriaDron {
  bool imuListo;
  bool baroListo;
  bool dronArmado;
};
bool dronImuListo = false;
bool dronBaroListo = false;
bool dronArmadoReal = false;
unsigned long tUltTelemetria = 0;

// ================================================================
//  EXPO Y RATES — muy suave para principiante
// ================================================================
#define EXPO_ROLL    0.80f
#define EXPO_PITCH   0.80f
#define EXPO_YAW     0.70f
#define RATE_ROLL    0.45f
#define RATE_PITCH   0.45f
#define RATE_YAW     0.35f
#define ZONA_MUERTA  60        // ADC units (~1.5% del rango)
#define THR_DEADZONE 80        // Zona muerta alrededor del centro del throttle
#define THROTTLE_RAMP 6.0f    // Mas rapido para que se sienta mas responsivo
#define ADC_SAMPLES  4         // Promedio de lecturas para reducir ruido ADC

// ─── ESTADO ──────────────────────────────────────────────────────
bool     armado     = false;
bool     connOK     = false;
uint8_t  modoActual = 0;      // 0=STABILIZE, 3=LOITER
uint16_t modoSeq    = 0;
float    throttleSuave = 1000.f;
unsigned long tUltTxOK   = 0;
uint16_t      txFailStreak = 0;
#define LINK_TIMEOUT_DISARM_MS  3000

const char* nombreModo[4] = {"STABILIZE","---","---","LOITER"};

// ─── CALIBRACIÓN — incluye throttle ──────────────────────────────
int centroJ1X = 2048;
int centroJ1Y = 2048;   // Centro del throttle (rest position)
int minJ1X    = 400,  maxJ1X = 3700;   // Yaw
int minJ1Y    = 0,    maxJ1Y = 4095;   // Throttle min/max
int centroJ2X = 2048;
int minJ2X    = 400,  maxJ2X = 3700;   // Roll
int centroJ2Y = 2048;
int minJ2Y    = 400,  maxJ2Y = 3700;   // Pitch

// ─── DEBOUNCE BTN_ARM ────────────────────────────────────────────
bool rawArmPrev   = HIGH, debArm   = HIGH;
bool rawEstopPrev = HIGH, debEstop = HIGH;
unsigned long tArmCambio   = 0;
unsigned long tEstopCambio = 0;
unsigned long tArmPress    = 0;
unsigned long tUltArmEvt   = 0;
bool armPressActiva = false;
bool modeIndicado   = false;   // para parpadeo de LED en hold largo

#define DEBOUNCE_MS        45
#define ARM_HOLD_MS       180
#define ARM_EVENT_LOCK_MS 500
#define MODE_HOLD_MS     3000   // 3 segundos para cambiar modo

// ─── LED blink ───────────────────────────────────────────────────
unsigned long tBlink  = 0;
int           blinkCnt = 0;
bool          ledState = false;
#define BLINK_ON_MS    120
#define BLINK_OFF_MS   120
#define BLINK_PAUSE_MS 800

// ================================================================
//  CALLBACKS ESP-NOW
// ================================================================
void onEnviado(const wifi_tx_info_t *info, esp_now_send_status_t status){
  bool ok = (status == ESP_NOW_SEND_SUCCESS);
  if(ok){ tUltTxOK = millis(); txFailStreak = 0; }
  else if(txFailStreak < 65535) txFailStreak++;
  connOK = (millis() - tUltTxOK) < 500;
}

void onRecibido(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  if(len == sizeof(TelemetriaDron)){
    TelemetriaDron tel;
    memcpy(&tel, data, sizeof(TelemetriaDron));
    dronImuListo = tel.imuListo;
    dronBaroListo = tel.baroListo;
    dronArmadoReal = tel.dronArmado;
    tUltTelemetria = millis();
  }
}

// ================================================================
//  ADC: lectura promediada para reducir ruido
//  El joystick es 0-1023 (512 = centro) pero ESP32 ADC es 12-bit
//  => 0-4095, centro ~2048, pero el real se calibra
// ================================================================
int adcRead(int pin){
  long suma = 0;
  for(int i = 0; i < ADC_SAMPLES; i++) suma += analogRead(pin);
  return (int)(suma / ADC_SAMPLES);
}

// ================================================================
//  EXPO Y RATES
// ================================================================
float applyExpo(float x, float e){
  return e*(x*x*x) + (1.f-e)*x;
}

int16_t leerStick(int pin, int centro, int minRaw, int maxRaw,
                  float expoVal, float rate){
  int raw = adcRead(pin);  // Usa promedio para reducir ruido
  raw = constrain(raw, minRaw, maxRaw);
  if(abs(raw - centro) < ZONA_MUERTA) return 0;
  float norm;
  int rN = (centro - ZONA_MUERTA) - minRaw;
  int rP = maxRaw - (centro + ZONA_MUERTA);
  if(raw < centro && rN > 0)
    norm = -(float)((centro - ZONA_MUERTA) - raw) / (float)rN;
  else if(raw > centro && rP > 0)
    norm =  (float)(raw - (centro + ZONA_MUERTA)) / (float)rP;
  else return 0;
  norm = constrain(norm, -1.f, 1.f);
  norm = applyExpo(norm, expoVal) * rate;
  return (int16_t)(norm * 500.f);
}

// ================================================================
//  CALIBRACIÓN — SIEMPRE al inicio, incluye throttle
// ================================================================
void calibrarJoysticks(){
  Serial.println("\n[CAL] ═══════════════════════════════════════");
  Serial.println("[CAL] FASE 0: NO TOQUES NADA (capturando posicion de reposo de todos los ejes)");
  tone(BUZZER, 800, 200);
  delay(2000);

  // Capturar posicion de reposo con 200 muestras promediadas (reduce ruido ADC)
  long sThr=0, sYaw=0, sRoll=0, sPitch=0;
  for(int i=0; i<200; i++){
    sThr   += analogRead(JOY1_Y);
    sYaw   += analogRead(JOY1_X);
    sRoll  += analogRead(JOY2_X);
    sPitch += analogRead(JOY2_Y);
    delay(5);
  }
  centroJ1Y = (int)(sThr/200);
  centroJ1X = (int)(sYaw/200);
  centroJ2X = (int)(sRoll/200);
  centroJ2Y = (int)(sPitch/200);
  Serial.printf("[CAL] Centros reposo: Thr=%d Yaw=%d Roll=%d Pitch=%d\n",
                centroJ1Y, centroJ1X, centroJ2X, centroJ2Y);

  // Inicializar rangos en el centro
  minJ1X = maxJ1X = centroJ1X;
  minJ1Y = maxJ1Y = centroJ1Y;
  minJ2X = maxJ2X = centroJ2X;
  minJ2Y = maxJ2Y = centroJ2Y;

  tone(BUZZER,1200,150); delay(200); tone(BUZZER,1600,150);
  Serial.println("[CAL] FASE 1: Mueve TODOS los sticks a TODOS los extremos (6s)");
  Serial.println("[CAL] Throttle: arriba todo, abajo todo. Yaw/Roll/Pitch: circulos completos");

  unsigned long tFin = millis() + 6000;
  while(millis() < tFin){
    int r1x=analogRead(JOY1_X), r1y=analogRead(JOY1_Y);
    int r2x=analogRead(JOY2_X), r2y=analogRead(JOY2_Y);
    if(r1x<minJ1X) minJ1X=r1x;  if(r1x>maxJ1X) maxJ1X=r1x;
    if(r1y<minJ1Y) minJ1Y=r1y;  if(r1y>maxJ1Y) maxJ1Y=r1y;
    if(r2x<minJ2X) minJ2X=r2x;  if(r2x>maxJ2X) maxJ2X=r2x;
    if(r2y<minJ2Y) minJ2Y=r2y;  if(r2y>maxJ2Y) maxJ2Y=r2y;
    delay(5);
  }

  Serial.printf("[CAL] Yaw:   %d — [%d] — %d (rango %d)\n",
                minJ1X,centroJ1X,maxJ1X,maxJ1X-minJ1X);
  Serial.printf("[CAL] Thr:   %d — [%d reposo] — %d\n",minJ1Y,centroJ1Y,maxJ1Y);
  bool thrInv = (minJ1Y > centroJ1Y);
  int thrRangeUp = thrInv ? (centroJ1Y - minJ1Y) : (maxJ1Y - centroJ1Y);
  Serial.printf("[CAL]   => Invertido:%s  Rango util arriba: %d\n",
                thrInv?"SI":"NO", thrRangeUp);
  Serial.printf("[CAL] Roll:  %d — [%d] — %d (rango %d)\n",
                minJ2X,centroJ2X,maxJ2X,maxJ2X-minJ2X);
  Serial.printf("[CAL] Pitch: %d — [%d] — %d (rango %d)\n",
                minJ2Y,centroJ2Y,maxJ2Y,maxJ2Y-minJ2Y);

  bool ok = (maxJ1X-minJ1X)>=300 && (maxJ2X-minJ2X)>=300 &&
            (maxJ2Y-minJ2Y)>=300 && thrRangeUp>=300;
  if(!ok){
    Serial.println("[CAL] ADVERTENCIA: rango pequeño — mueve mas los sticks");
    tone(BUZZER,400,200); delay(250); tone(BUZZER,400,200);
  } else {
    Serial.println("[CAL] OK — calibracion exitosa");
    tone(BUZZER,1800,150); delay(200); tone(BUZZER,2200,200);
  }
  throttleSuave = 1000.f;
  Serial.println("[CAL] ═══════════════════════════════════════\n");
}


// ================================================================
//  LED — 1 parpadeo=STABILIZE, 2 parpadeos=LOITER
// ================================================================
void actualizarLEDModo(){
  if(!armado){ digitalWrite(LED_MODE, LOW); blinkCnt=0; ledState=false; return; }
  int numBlinks = (modoActual==0) ? 1 : 2;
  unsigned long ahora = millis();
  if(blinkCnt < numBlinks){
    if(!ledState){
      if((ahora-tBlink) >= BLINK_OFF_MS){ ledState=true; digitalWrite(LED_MODE,HIGH); tBlink=ahora; }
    } else {
      if((ahora-tBlink) >= BLINK_ON_MS){ ledState=false; digitalWrite(LED_MODE,LOW); tBlink=ahora; blinkCnt++; }
    }
  } else {
    if((ahora-tBlink) >= BLINK_PAUSE_MS){ blinkCnt=0; tBlink=ahora; }
  }
}

// ================================================================
//  SETUP
// ================================================================
void setup(){
  Serial.begin(115200);
  delay(500);
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║  CONTROL F450 — FIRMWARE V3.3        ║");
  Serial.println("╚══════════════════════════════════════╝");

  pinMode(BTN_ARM,   INPUT_PULLUP);
  pinMode(BTN_ESTOP, INPUT_PULLUP);
  pinMode(LED_ARM,   OUTPUT);
  pinMode(LED_MODE,  OUTPUT);
  pinMode(LED_CONN,  OUTPUT);
  pinMode(BUZZER,    OUTPUT);

  rawArmPrev   = debArm   = digitalRead(BTN_ARM);
  rawEstopPrev = debEstop = digitalRead(BTN_ESTOP);

  // Test LEDs
  for(int p : {LED_ARM, LED_MODE, LED_CONN}){
    digitalWrite(p, HIGH); delay(150); digitalWrite(p, LOW);
  }
  tone(BUZZER,1000,100); delay(150); tone(BUZZER,1500,150);

  // Calibración SIEMPRE — sin opción de skip
  Serial.println("[CAL] Calibracion obligatoria al inicio...");
  calibrarJoysticks();

  txDatos = {1000, 0, 0, 0, false, false, 0, 0};

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  // Configuracion WiFi para ESP-NOW — protocolo estándar
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  // Protocolo estándar 11b/g/n: menor latencia que LR
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(84); // Maxima potencia (el control no sufre brownout)
  Serial.printf("[NET] MAC Control: %s\n", WiFi.macAddress().c_str());

  if(esp_now_init() != ESP_OK){
    Serial.println("[NET] ERROR ESP-NOW");
    while(1){ tone(BUZZER,300,500); delay(600); }
  }
  esp_now_register_send_cb(onEnviado);
  esp_now_register_recv_cb(onRecibido);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, macDron, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if(esp_now_add_peer(&peer) != ESP_OK)
    Serial.println("[NET] ERROR peer");
  else
    Serial.printf("[NET] Dron: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  macDron[0],macDron[1],macDron[2],macDron[3],macDron[4],macDron[5]);

  Serial.println("\n[OK] Control listo.");
  Serial.println("[BTN] ARM hold 180ms    = armar/desarmar");
  Serial.println("[BTN] ARM hold 3000ms   = cambiar modo STABILIZE<->LOITER");
  Serial.println("[BTN] ESTOP             = corte inmediato");
  Serial.println("[Serial] c=recalibrar m=MAC v=valores");
  tUltTxOK = millis();
  tone(BUZZER, 1700, 400);
}

// ================================================================
//  LOOP
// ================================================================
unsigned long tEnvio = 0;
unsigned long tPrint = 0;

void loop(){
  unsigned long ahora = millis();

  // ── Throttle mapping directo (rango completo con auto-inversion) ──
  // Joystick: 0-1023 nativo (512=centro) → ESP32 ADC 12-bit: 0-4095
  int rawThr = adcRead(JOY1_Y);
  float tgtThr;
  
  // Si el punto de reposo (centro) esta mas cerca de maxJ1Y, significa
  // que el reposo es arriba (o abajo invertido). Asumimos que el 
  // joystick tiene un resorte al centro o se calibro empezando al centro.
  // Pero la intencion del usuario es un rango completo.
  // Si el ADC baja al subir el stick (invertido):
  bool thrInv = (centroJ1Y > (minJ1Y + maxJ1Y)/2); // Metodo mas seguro

  if(thrInv){
    tgtThr = (float)map(constrain(rawThr, minJ1Y, maxJ1Y), minJ1Y, maxJ1Y, 2000, 1000);
  } else {
    tgtThr = (float)map(constrain(rawThr, minJ1Y, maxJ1Y), minJ1Y, maxJ1Y, 1000, 2000);
  }

  if(throttleSuave < tgtThr)
    throttleSuave = min(throttleSuave + THROTTLE_RAMP, tgtThr);
  else
    throttleSuave = max(throttleSuave - THROTTLE_RAMP, tgtThr);
  txDatos.throttle = (int16_t)constrain(throttleSuave, 1000, 2000);

  // ── Sticks con expo y ADC promediado ─────────────────────────
  txDatos.yaw   = leerStick(JOY1_X, centroJ1X, minJ1X, maxJ1X, EXPO_YAW,   RATE_YAW);
  txDatos.pitch = -leerStick(JOY2_Y, centroJ2Y, minJ2Y, maxJ2Y, EXPO_PITCH,  RATE_PITCH); // INVERTIDO: Para que empujar arriba (enfrente) baje la nariz
  txDatos.roll  = leerStick(JOY2_X, centroJ2X, minJ2X, maxJ2X, EXPO_ROLL,   RATE_ROLL);

  // ── Debounce ──────────────────────────────────────────────────
  bool bAr = digitalRead(BTN_ARM);
  bool bEs = digitalRead(BTN_ESTOP);

  if(bAr != rawArmPrev){ rawArmPrev = bAr; tArmCambio = ahora; }
  if(bEs != rawEstopPrev){ rawEstopPrev = bEs; tEstopCambio = ahora; }

  // ── BTN ARM — doble función según duración del hold ───────────
  if((ahora - tArmCambio) >= DEBOUNCE_MS && debArm != bAr){
    debArm = bAr;
    if(debArm == LOW){
      // Presionado
      armPressActiva = true;
      tArmPress = ahora;
      modeIndicado = false;
    } else {
      // Soltado
      if(armPressActiva){
        armPressActiva = false;
        unsigned long holdMs = ahora - tArmPress;

        if(holdMs >= MODE_HOLD_MS){
          // ── HOLD 3s: cambiar modo ────────────────────────────
          if(armado){
            modoActual = (modoActual == 0) ? 3 : 0;
            modoSeq++;
            blinkCnt = 0; tBlink = ahora; ledState = false;
            if(modoActual == 0){
              tone(BUZZER,1200,150);
              Serial.printf("[MODO] STABILIZE (seq=%u)\n", modoSeq);
            } else {
              tone(BUZZER,1400,120); delay(150); tone(BUZZER,1800,180);
              Serial.printf("[MODO] LOITER (seq=%u) — espera confirmacion del dron\n", modoSeq);
            }
          } else {
            tone(BUZZER,400,200);
            Serial.println("[MODO] Armar primero para cambiar modo");
          }

        } else if(holdMs >= ARM_HOLD_MS && (ahora - tUltArmEvt) >= ARM_EVENT_LOCK_MS){
          // ── HOLD 180ms-2999ms: armar/desarmar ────────────────
          tUltArmEvt = ahora;
          if(!armado){
            if(txDatos.throttle < 1080){
              armado = true;
              modoActual = 0;
              blinkCnt = 0; tBlink = ahora; ledState = false;
              digitalWrite(LED_ARM, HIGH);
              tone(BUZZER,1400,80); delay(100); tone(BUZZER,1800,120);
              Serial.println("[ARM] ARMADO — STABILIZE");
            } else {
              Serial.println("[ARM] Baja THROTTLE al minimo primero");
              tone(BUZZER,400,400);
            }
          } else {
            armado = false;
            modoActual = 0;
            blinkCnt = 0; ledState = false;
            digitalWrite(LED_ARM,  LOW);
            digitalWrite(LED_MODE, LOW);
            tone(BUZZER,1000,200);
            Serial.println("[ARM] DESARMADO");
          }
        }
        // holdMs < ARM_HOLD_MS → click rápido, ignorar
      }
    }
  }

  // ── Indicador LED durante hold largo (avisa que ya puede soltar) ──
  // Cuando lleva más de 2.5s presionado, LED_MODE parpadea rápido
  if(armPressActiva && (ahora - tArmPress) >= 2500 && !modeIndicado){
    modeIndicado = true;
    // Parpadeo rápido para avisar "ya puede soltar para cambiar modo"
    for(int i=0;i<3;i++){
      digitalWrite(LED_MODE,HIGH); delay(80);
      digitalWrite(LED_MODE,LOW);  delay(80);
    }
    tone(BUZZER, 1600, 200);
  }

  // ── BTN ESTOP ─────────────────────────────────────────────────
  if((ahora - tEstopCambio) >= DEBOUNCE_MS && debEstop != bEs){
    debEstop = bEs;
    if(debEstop == LOW){
      armPressActiva = false;
      modeIndicado   = false;
      armado         = false;
      modoActual     = 0;
      blinkCnt       = 0;
      ledState       = false;
      digitalWrite(LED_ARM,  LOW);
      digitalWrite(LED_MODE, LOW);
      tone(BUZZER, 200, 800);
      Serial.println("[!!!] EMERGENCY STOP");
    }
  }

  // ── Struct TX ─────────────────────────────────────────────────
  txDatos.emergencyStop = (debEstop == LOW);
  txDatos.armado        = armado;
  txDatos.flightMode    = modoActual;
  txDatos.modeSeq       = modoSeq;

  // ── LED modo ──────────────────────────────────────────────────
  actualizarLEDModo();

  // ── Enviar 50Hz ───────────────────────────────────────────────
  if(ahora - tEnvio >= 20){
    esp_now_send(macDron, (uint8_t*)&txDatos, sizeof(txDatos));
    tEnvio = ahora;
  }

  // ── LED CONN inteligente (Telemetria) ─────────────────────────
  bool telemetriaViva = (ahora - tUltTelemetria) < 2000;
  if(connOK && telemetriaViva && dronImuListo && dronBaroListo){
    digitalWrite(LED_CONN, HIGH); // Todo perfecto y listo para volar
  } else if (connOK) {
    digitalWrite(LED_CONN, (ahora/250)%2); // Parpadeo: conectado pero sensores fallan
  } else {
    digitalWrite(LED_CONN, LOW); // Desconectado
  }

  // ── Auto-desarmar si link perdido por mucho tiempo ─────────────
  if(armado && txFailStreak >= 150 && (ahora - tUltTxOK) > 5000){
    armado     = false;
    modoActual = 0;
    blinkCnt   = 0;
    ledState   = false;
    txDatos.armado     = false;
    txDatos.flightMode = 0;
    digitalWrite(LED_ARM,  LOW);
    digitalWrite(LED_MODE, LOW);
    tone(BUZZER, 350, 500);
    Serial.println("[SAFE] Enlace perdido — DESARMADO");
    tUltTxOK = ahora;
  }

  // ── Sync Armado Real con Dron ─────────────────────────────────
  if(armado && telemetriaViva && !dronArmadoReal && (ahora - tArmPress) > 2000){
    // El control cree estar armado, pero el dron lo rechazó!
    armado = false;
    modoActual = 0;
    blinkCnt = 0; ledState = false;
    digitalWrite(LED_ARM, LOW);
    digitalWrite(LED_MODE, LOW);
    tone(BUZZER, 300, 300); delay(350); tone(BUZZER, 300, 300);
    Serial.println("[ARM] RECHAZADO por el dron (Acelerador no al minimo o sensores no listos)");
  }

  // ── Serial 5Hz ────────────────────────────────────────────────
  if(ahora - tPrint >= 200){
    tPrint = ahora;
    Serial.printf("T:%4d Y:%5d P:%5d R:%5d | ARM:%d MODO:%-10s | %s%s\n",
                  txDatos.throttle, txDatos.yaw, txDatos.pitch, txDatos.roll,
                  txDatos.armado,
                  nombreModo[modoActual],
                  connOK ? "TX:OK" : "TX:--",
                  armPressActiva ?
                    (ahora-tArmPress>=2500?" [SUELTA=MODO]":" [HOLD...]") : "");
  }

  // ── Comandos serial ───────────────────────────────────────────
  if(Serial.available()){
    char c = Serial.read();
    if(c == 'c'){
      Serial.println("[CAL] Recalibrando...");
      calibrarJoysticks();
    }
    if(c == 'm') Serial.println(WiFi.macAddress());
    if(c == 'v'){
      Serial.printf("[V] Throttle: ADC_raw=%d  centro=%d  max=%d  TX=%d\n",
                    analogRead(JOY1_Y), centroJ1Y, maxJ1Y, txDatos.throttle);
      Serial.printf("[V] Yaw=%d Roll=%d Pitch=%d\n",
                    txDatos.yaw, txDatos.roll, txDatos.pitch);
      Serial.printf("[V] ARM:%s MODO:%s CONN:%s\n",
                    armado?"SI":"NO", nombreModo[modoActual], connOK?"OK":"--");
    }
  }
}