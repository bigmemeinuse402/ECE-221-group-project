#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <BluetoothA2DPSink.h>
#include <cstring>

// ============================================================================
// SECTION 1: CONFIGURATION (MATCHED TO 120-SAMPLE FEATHER S3 WALKIE-TALKIE)
// ============================================================================
namespace Config {
  constexpr int LED_PIN          = 2;   // Onboard blue LED on Hoyond DevKit
  constexpr uint32_t MAGIC_ID    = 0x4D413236; // Must match Feather S3 ("MA26")
  constexpr uint8_t WIFI_CHANNEL = 11;         // Must match Feather S3 channel

  constexpr int AUDIO_SAMPLES    = 120;        // Matched to 120-sample Feather code
  constexpr unsigned long DISCOVER_INTERVAL_MS = 1500;
}

// ============================================================================
// SECTION 2: PACKET STRUCTURE & SHARED STATE
// ============================================================================
enum PacketType : uint8_t {
  PKT_DISCOVER = 1,
  PKT_PAIR_ACK = 2,
  PKT_AUDIO    = 3
};

struct __attribute__((packed)) WalkiePacket {
  uint32_t magic;
  uint8_t type;
  uint32_t sequence;
  int16_t samples[Config::AUDIO_SAMPLES];
};

BluetoothA2DPSink a2dp_sink;

namespace State {
  uint8_t broadcastMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t peerMAC[6]      = {0};
  bool isPaired           = false;
  uint32_t txSequence     = 0;

  int16_t txAccum[Config::AUDIO_SAMPLES];
  int accumCount          = 0;
  float samplePhase       = 0.0f;
  unsigned long lastMusicMs = 0;
}

// ============================================================================
// SECTION 3: ESP-NOW WIRELESS MODULE
// ============================================================================
namespace WirelessModule {
  WalkiePacket outgoingPacket;

  bool registerPeer(const uint8_t* mac) {
    if (esp_now_is_peer_exist(mac)) return true;
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = Config::WIFI_CHANNEL;
    peerInfo.encrypt = false;
    return (esp_now_add_peer(&peerInfo) == ESP_OK);
  }

  void lockOntoPeer(const uint8_t* mac) {
    bool isNew = (!State::isPaired || memcmp(State::peerMAC, mac, 6) != 0);
    if (registerPeer(mac)) {
      memcpy(State::peerMAC, mac, 6);
      if (isNew) {
        State::isPaired = true;
        Serial.printf("[PAIRED] Linked to Feather %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      }
    }
  }

  void sendControlPacket(const uint8_t* targetMAC, PacketType type) {
    WalkiePacket ctrlPkt = {};
    ctrlPkt.magic = Config::MAGIC_ID;
    ctrlPkt.type = type;
    esp_now_send(targetMAC, (uint8_t*)&ctrlPkt, sizeof(ctrlPkt));
  }

  void sendAudioFrame(const int16_t* samples) {
    if (!State::isPaired) return;
    outgoingPacket.magic = Config::MAGIC_ID;
    outgoingPacket.type = PKT_AUDIO;
    outgoingPacket.sequence = State::txSequence++;
    memcpy(outgoingPacket.samples, samples, sizeof(outgoingPacket.samples));
    esp_now_send(State::peerMAC, (uint8_t*)&outgoingPacket, sizeof(outgoingPacket));
  }

  void onPacketSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {}

  void onPacketReceived(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len != sizeof(WalkiePacket)) return;

    const WalkiePacket* incoming = reinterpret_cast<const WalkiePacket*>(data);
    if (incoming->magic != Config::MAGIC_ID) return;

    if (incoming->type == PKT_DISCOVER) {
      lockOntoPeer(info->src_addr);
      sendControlPacket(State::peerMAC, PKT_PAIR_ACK);
    } else if (incoming->type == PKT_PAIR_ACK) {
      lockOntoPeer(info->src_addr);
    }
  }

  void init() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    esp_wifi_set_channel(Config::WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
      Serial.println("[ERROR] ESP-NOW init failed!");
      while (true) delay(1000);
    }

    esp_now_register_send_cb(onPacketSent);
    esp_now_register_recv_cb(onPacketReceived);
    registerPeer(State::broadcastMAC);
  }
}

// ============================================================================
// SECTION 4: PHONE BLUETOOTH MUSIC -> 16kHz MONO 120-SAMPLE CONVERTER
// ============================================================================
void onBluetoothAudioData(const uint8_t* data, uint32_t length) {
  if (!State::isPaired || length < 4) return;

  const int16_t* stereoSamples = reinterpret_cast<const int16_t*>(data);
  uint32_t stereoFrames = length / 4; // 4 bytes per stereo frame (16-bit L + 16-bit R)

  // Downsample phone's 44.1kHz stereo audio to 16kHz mono (ratio 44100 / 16000 = 2.75625)
  const float step = 44100.0f / 16000.0f;

  while (State::samplePhase < (float)stereoFrames) {
    uint32_t idx = (uint32_t)State::samplePhase;
    int32_t left  = stereoSamples[idx * 2];
    int32_t right = stereoSamples[idx * 2 + 1];
    int16_t mono  = (int16_t)((left + right) / 2);

    State::txAccum[State::accumCount++] = mono;
    if (State::accumCount >= Config::AUDIO_SAMPLES) {
      WirelessModule::sendAudioFrame(State::txAccum);
      State::accumCount = 0;
      State::lastMusicMs = millis();
    }
    State::samplePhase += step;
  }
  State::samplePhase -= (float)stereoFrames;
}

// ============================================================================
// SECTION 5: SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(Config::LED_PIN, OUTPUT);

  // Start ESP-NOW first, then start A2DP Bluetooth Audio Receiver (no I2S hardware needed)
  WirelessModule::init();

  a2dp_sink.set_stream_reader(onBluetoothAudioData, false);
  a2dp_sink.set_auto_reconnect(true);
  a2dp_sink.start("ESP32_Walkie_Music");

  Serial.println("[BT] Ready! Pair your phone to 'ESP32_Walkie_Music' in Bluetooth Settings.");
}

void loop() {
  // 1. Auto-pair with the Feather S3 if not connected yet
  if (!State::isPaired) {
    static unsigned long lastPing = 0;
    if (millis() - lastPing >= Config::DISCOVER_INTERVAL_MS) {
      lastPing = millis();
      digitalWrite(Config::LED_PIN, !digitalRead(Config::LED_PIN));
      WirelessModule::sendControlPacket(State::broadcastMAC, PKT_DISCOVER);
    }
    return;
  }

  // 2. Solid blue LED while streaming phone music; off when idle
  digitalWrite(Config::LED_PIN, (millis() - State::lastMusicMs < 200) ? HIGH : LOW);
}