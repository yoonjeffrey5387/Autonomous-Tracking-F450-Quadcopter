// ============================================================================
//  RadioLink.h — ESP-NOW RX/TX
//    · Aprendizaje automático del peer (control remoto)
//    · Copia atómica del RX struct (no se accede al buffer parcial)
//    · Timestamps independientes para warning / cut failsafe
//    · TX de telemetría periódica
// ============================================================================
#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "Config.h"
#include "Types.h"

class RadioLink {
public:
  // Estado RX
  DatosControl rxLatest{};
  uint32_t     lastRxMs        = 0;
  uint32_t     packetsReceived = 0;
  bool         peerLearned     = false;
  uint8_t      peerMac[6]      = {0};

  bool begin() {
    if (instance_) return false;          // singleton
    instance_ = this;
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(&RadioLink::onRecvStatic);
    esp_now_register_send_cb(&RadioLink::onSendStatic);
    return true;
  }

  // Devuelve una copia atómica del último paquete.
  DatosControl snapshot() {
    noInterrupts();
    DatosControl c = rxLatest;
    interrupts();
    return c;
  }

  // True si la comunicación está viva (< warn).
  bool linkHealthy() const {
    return (millis() - lastRxMs) < FAILSAFE_WARN_MS;
  }
  // True si hay que CORTAR motores (sin paquetes prolongados).
  bool failsafeCut() const {
    return (millis() - lastRxMs) >= FAILSAFE_CUT_MS;
  }

  bool sendTelemetry(const TelemetriaDron& t) {
    if (!peerLearned) return false;
    esp_err_t r = esp_now_send(peerMac, (const uint8_t*)&t, sizeof(t));
    return r == ESP_OK;
  }

private:
  static RadioLink* instance_;

  static void onRecvStatic(const esp_now_recv_info_t* info,
                           const uint8_t* data, int len) {
    if (!instance_) return;
    instance_->handleRecv(info, data, len);
  }
  static void onSendStatic(const wifi_tx_info_t* /*info*/, esp_now_send_status_t /*st*/) {}

  void handleRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len < (int)sizeof(DatosControl)) return;
    DatosControl tmp;
    memcpy(&tmp, data, sizeof(DatosControl));
    // copia atómica al estado público
    noInterrupts();
    rxLatest = tmp;
    interrupts();
    lastRxMs = millis();
    packetsReceived++;

    if (!peerLearned && info && info->src_addr) {
      memcpy(peerMac, info->src_addr, 6);
      esp_now_peer_info_t p{};
      memcpy(p.peer_addr, peerMac, 6);
      p.channel = ESPNOW_CHANNEL;
      p.encrypt = false;
      esp_now_add_peer(&p);
      peerLearned = true;
    }
  }
};

inline RadioLink* RadioLink::instance_ = nullptr;
