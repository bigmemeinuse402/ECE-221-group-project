#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/i2s.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_MAX1704X.h>
#include <cstring>

// ============================================================================
// SECTION 1: HARDWARE & CONFIGURATION (ADAFRUIT ESP32-S3 FEATHER)
// ============================================================================
namespace Config {
  constexpr int LED_PIN          = 13;  // Built-in red LED
  constexpr int PTT_BUTTON_PIN   = 11;  // PTT button between 11 and GND

  // 1.8" TFT SPI LCD (ST7735)
  constexpr int TFT_CS_PIN       = 10;
  constexpr int TFT_DC_PIN       = 9;
  constexpr int TFT_RST_PIN      = 6;
  constexpr int TFT_MOSI_PIN     = 35;  // MO
  constexpr int TFT_SCLK_PIN     = 36;  // SCK
  constexpr int TFT_BLK_PIN      = -1;  // Wired to 3V

  // I2S Microphone (INMP441)
  constexpr int MIC_BCLK_PIN     = 14;  // A4
  constexpr int MIC_WS_PIN       = 15;  // A3
  constexpr int MIC_DATA_PIN     = 16;  // A2

  // I2S Speaker Amplifier (MAX98357A)
  constexpr int SPK_BCLK_PIN     = 18;  // A0
  constexpr int SPK_WS_PIN       = 17;  // A1
  constexpr int SPK_DATA_PIN     = 8;   // A5

  constexpr uint32_t MAGIC_ID    = 0x4D413236;
  constexpr uint8_t WIFI_CHANNEL = 11;

  constexpr uint8_t TFT_TAB      = INITR_BLACKTAB;
  constexpr uint8_t TFT_ROTATION = 1;
  constexpr int SCREEN_WIDTH     = 160;
  constexpr int SCREEN_HEIGHT    = 128;

  constexpr int TOP_BAR_H        = 26;
  constexpr int BOTTOM_BAR_H     = 26;
  constexpr int WAVE_TOP_Y       = TOP_BAR_H + 1;
  constexpr int WAVE_BOTTOM_Y    = SCREEN_HEIGHT - BOTTOM_BAR_H - 2;

  constexpr int WAVE_TARGET_FPS  = 12;
  constexpr int WAVE_AMPLITUDE   = 10000;

  constexpr uint16_t COLOR_BG          = ST77XX_BLACK;
  constexpr uint16_t COLOR_TOP_BG      = 0x1082;
  constexpr uint16_t COLOR_TOP_TEXT    = ST77XX_WHITE;
  constexpr uint16_t COLOR_CENTERLINE  = 0x18E3;
  constexpr uint16_t COLOR_RX_WAVE     = ST77XX_CYAN;
  constexpr uint16_t COLOR_TX_WAVE     = ST77XX_ORANGE;
  constexpr uint16_t COLOR_SEARCH      = ST77XX_YELLOW;
  constexpr uint16_t COLOR_READY       = ST77XX_GREEN;
  constexpr uint16_t COLOR_LOW_BAT     = ST77XX_RED;

  // 120 samples @ 16kHz = 7.5ms per packet (240 bytes, safely under ESP-NOW 250B limit)
  constexpr int SAMPLE_RATE      = 16000;
  constexpr int AUDIO_SAMPLES    = 120;
  constexpr int JITTER_SLOTS     = 16;  // 16-slot ring buffer (~120ms anti-chop cushion)
  constexpr int MIC_DIGITAL_GAIN = 4;

  constexpr unsigned long DISCOVER_INTERVAL_MS   = 1500;
  constexpr unsigned long RX_HOLD_TIMEOUT_MS     = 250;
  constexpr unsigned long PAIR_FLASH_DURATION_MS = 1800;
  constexpr unsigned long BAT_READ_INTERVAL_MS   = 5000;
}

// ============================================================================
// SECTION 2: DATA STRUCTURES & LOCK-FREE JITTER RING BUFFER
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

namespace State {
  uint8_t broadcastMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t peerMAC[6]      = {0};
  bool isPaired           = false;
  uint32_t packetSequence = 0;

  bool isTransmitting          = false;
  unsigned long lastRxTime     = 0;
  unsigned long pairedAtTime   = 0;
  volatile bool peerMacUpdated = false;
  int batteryPercent           = 100;

  // Lock-free RX Jitter Ring Buffer (written by Wi-Fi ISR, drained by loop)
  int16_t rxRing[Config::JITTER_SLOTS][Config::AUDIO_SAMPLES];
  volatile uint8_t rxHead = 0;
  volatile uint8_t rxTail = 0;
  bool prebuffering       = true;

  int16_t waveSnapshot[Config::AUDIO_SAMPLES] = {0};
  volatile bool newWaveAvailable = false;
}

// ============================================================================
// SECTION 3: BATTERY MONITOR MODULE
// ============================================================================
namespace BatteryModule {
  Adafruit_MAX17048 maxlipo;
  bool gaugeFound = false;

  int readPercentage() {
    if (!gaugeFound) return 100;
    float pct = maxlipo.cellPercent();
    if (isnan(pct)) return State::batteryPercent;
    return constrain((int)round(pct), 0, 100);
  }

  void init() {
    Wire.begin();
    gaugeFound = maxlipo.begin();
    State::batteryPercent = readPercentage();
  }

  void update() {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck >= Config::BAT_READ_INTERVAL_MS) {
      lastCheck = millis();
      State::batteryPercent = readPercentage();
    }
  }
}

// ============================================================================
// SECTION 4: DISPLAY & 3-ZONE HUD RENDERER MODULE
// ============================================================================
namespace DisplayModule {
  Adafruit_ST7735 tft(Config::TFT_CS_PIN, Config::TFT_DC_PIN, Config::TFT_RST_PIN);
  int16_t previousY[Config::SCREEN_WIDTH];
  String lastTopText    = "";
  int lastDrawnBatPct   = -1;
  String lastBottomText = "";

  String formatMACText(const uint8_t* mac, bool valid) {
    if (!valid) return "SCANNING CH " + String(Config::WIFI_CHANNEL);
    char buf[20];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
  }

  void init() {
    if (Config::TFT_BLK_PIN >= 0) {
      pinMode(Config::TFT_BLK_PIN, OUTPUT);
      digitalWrite(Config::TFT_BLK_PIN, HIGH);
    }
    SPI.begin(Config::TFT_SCLK_PIN, -1, Config::TFT_MOSI_PIN, Config::TFT_CS_PIN);
    tft.initR(Config::TFT_TAB);
    tft.setRotation(Config::TFT_ROTATION);
    tft.fillScreen(Config::COLOR_BG);

    int centerY = (Config::WAVE_TOP_Y + Config::WAVE_BOTTOM_Y) / 2;
    for (int x = 0; x < Config::SCREEN_WIDTH; x++) previousY[x] = centerY;
  }

  void drawTopBar() {
    String macText = formatMACText(State::peerMAC, State::isPaired);
    int batPct = State::batteryPercent;
    if (macText == lastTopText && batPct == lastDrawnBatPct && !State::peerMacUpdated) return;

    lastTopText = macText;
    lastDrawnBatPct = batPct;
    State::peerMacUpdated = false;

    tft.fillRect(0, 0, Config::SCREEN_WIDTH, Config::TOP_BAR_H - 1, Config::COLOR_TOP_BG);
    tft.drawFastHLine(0, Config::TOP_BAR_H - 1, Config::SCREEN_WIDTH, ST77XX_WHITE);

    tft.setTextColor(Config::COLOR_TOP_TEXT);
    tft.setTextSize(1);
    tft.setCursor(4, 9);
    tft.print(macText);

    uint16_t batColor = (batPct <= 20) ? Config::COLOR_LOW_BAT : Config::COLOR_READY;
    tft.drawRect(112, 8, 15, 9, ST77XX_WHITE);
    tft.fillRect(127, 10, 2, 5, ST77XX_WHITE);
    int fillW = map(batPct, 0, 100, 0, 11);
    if (fillW > 0) tft.fillRect(114, 10, fillW, 5, batColor);

    tft.setTextColor(batColor);
    tft.setCursor(132, 9);
    tft.print(batPct);
    tft.print("%");
  }

  void drawBottomBanner(const char* label, uint16_t bgColor) {
    if (lastBottomText == label) return;
    lastBottomText = label;

    int topY = Config::SCREEN_HEIGHT - Config::BOTTOM_BAR_H;
    tft.drawFastHLine(0, topY, Config::SCREEN_WIDTH, ST77XX_WHITE);
    tft.fillRect(0, topY + 1, Config::SCREEN_WIDTH, Config::BOTTOM_BAR_H - 1, bgColor);

    tft.setTextColor(ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(6, topY + 9);
    tft.print(label);
  }

  uint16_t updateHUDStatus() {
    drawTopBar();
    if (!State::isPaired) {
      drawBottomBanner("SEARCHING FOR ESP32...", Config::COLOR_SEARCH);
      return Config::COLOR_SEARCH;
    }
    if (millis() - State::pairedAtTime < Config::PAIR_FLASH_DURATION_MS) {
      drawBottomBanner("CONNECTED! LINK READY", Config::COLOR_READY);
      return Config::COLOR_READY;
    }
    if (State::isTransmitting) {
      drawBottomBanner("PTT ACTIVE: SENDING", Config::COLOR_TX_WAVE);
      return Config::COLOR_TX_WAVE;
    }
    if (millis() - State::lastRxTime < Config::RX_HOLD_TIMEOUT_MS) {
      drawBottomBanner("RX: RECEIVING AUDIO", Config::COLOR_RX_WAVE);
      return Config::COLOR_RX_WAVE;
    }
    drawBottomBanner("IDLE: WAITING FOR RX", Config::COLOR_READY);
    return Config::COLOR_RX_WAVE;
  }

  void update() {
    // Pause heavy SPI screen redraws while actively playing incoming voice
    bool isReceivingNow = (millis() - State::lastRxTime < Config::RX_HOLD_TIMEOUT_MS);
    if (isReceivingNow) return;

    static unsigned long lastFrameTime = 0;
    if (millis() - lastFrameTime < (1000 / Config::WAVE_TARGET_FPS)) return;
    lastFrameTime = millis();

    uint16_t waveColor = updateHUDStatus();

    int16_t samplesToDraw[Config::AUDIO_SAMPLES];
    if (State::newWaveAvailable && State::isTransmitting) {
      memcpy(samplesToDraw, State::waveSnapshot, sizeof(samplesToDraw));
      State::newWaveAvailable = false;
    } else {
      memset(samplesToDraw, 0, sizeof(samplesToDraw));
    }

    const int centerY      = (Config::WAVE_TOP_Y + Config::WAVE_BOTTOM_Y) / 2;
    const int maxAmplitude = (Config::WAVE_BOTTOM_Y - Config::WAVE_TOP_Y) / 2;
    tft.drawFastHLine(0, centerY, Config::SCREEN_WIDTH, Config::COLOR_CENTERLINE);

    int prevNewY = centerY;
    for (int x = 0; x < Config::SCREEN_WIDTH; x++) {
      int sampleIndex = (x * Config::AUDIO_SAMPLES) / Config::SCREEN_WIDTH;
      int32_t rawSample = samplesToDraw[sampleIndex];
      int yOffset = constrain((rawSample * maxAmplitude) / Config::WAVE_AMPLITUDE, -maxAmplitude, maxAmplitude);
      int newY = centerY - yOffset;

      if (x > 0) {
        tft.drawLine(x - 1, previousY[x - 1], x, previousY[x], Config::COLOR_BG);
        tft.drawLine(x - 1, prevNewY, x, newY, waveColor);
      }
      previousY[x - 1] = prevNewY;
      prevNewY = newY;
    }
    previousY[Config::SCREEN_WIDTH - 1] = prevNewY;
  }
}

// ============================================================================
// SECTION 5: I2S AUDIO HARDWARE MODULE (MICROPHONE & SPEAKER)
// ============================================================================
namespace AudioModule {
  constexpr i2s_port_t MIC_PORT = I2S_NUM_0;
  constexpr i2s_port_t SPK_PORT = I2S_NUM_1;
  int32_t rawMicBuffer[Config::AUDIO_SAMPLES];

  void initMicrophone() {
    i2s_config_t micConfig = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = Config::SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 6,
      .dma_buf_len = Config::AUDIO_SAMPLES,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0
    };

    i2s_pin_config_t micPins = {
      .bck_io_num = Config::MIC_BCLK_PIN,
      .ws_io_num = Config::MIC_WS_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = Config::MIC_DATA_PIN
    };

    i2s_driver_install(MIC_PORT, &micConfig, 0, NULL);
    i2s_set_pin(MIC_PORT, &micPins);
  }

  void initSpeaker() {
    i2s_config_t spkConfig = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = Config::SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 10,
      .dma_buf_len = Config::AUDIO_SAMPLES,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
    };

    i2s_pin_config_t spkPins = {
      .bck_io_num = Config::SPK_BCLK_PIN,
      .ws_io_num = Config::SPK_WS_PIN,
      .data_out_num = Config::SPK_DATA_PIN,
      .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(SPK_PORT, &spkConfig, 0, NULL);
    i2s_set_pin(SPK_PORT, &spkPins);
  }

  void init() {
    initMicrophone();
    initSpeaker();
  }

  bool captureMicFrame(int16_t* destinationBuffer) {
    size_t bytesRead = 0;
    i2s_read(MIC_PORT, rawMicBuffer, sizeof(rawMicBuffer), &bytesRead, portMAX_DELAY);
    if (bytesRead != sizeof(rawMicBuffer)) return false;

    for (int i = 0; i < Config::AUDIO_SAMPLES; i++) {
      int32_t scaledSample = (rawMicBuffer[i] >> 16) * Config::MIC_DIGITAL_GAIN;
      destinationBuffer[i] = (int16_t)constrain(scaledSample, -32768, 32767);
    }
    return true;
  }

  void drainJitterBufferToSpeaker() {
    uint8_t available = (State::rxHead - State::rxTail + Config::JITTER_SLOTS) % Config::JITTER_SLOTS;

    if (available == 0) {
      if (millis() - State::lastRxTime > 80) State::prebuffering = true;
      return;
    }

    // Wait until 3 packets (~22ms) accumulate before starting playback
    if (State::prebuffering) {
      if (available < 3) return;
      State::prebuffering = false;
    }

    while (State::rxTail != State::rxHead) {
      size_t bytesWritten = 0;
      i2s_write(SPK_PORT, State::rxRing[State::rxTail],
                Config::AUDIO_SAMPLES * sizeof(int16_t), &bytesWritten, pdMS_TO_TICKS(25));
      State::rxTail = (State::rxTail + 1) % Config::JITTER_SLOTS;
    }
  }

  void muteSpeaker() {
    State::rxTail = State::rxHead;
    State::prebuffering = true;
    i2s_zero_dma_buffer(SPK_PORT);
  }
}

// ============================================================================
// SECTION 6: ESP-NOW WIRELESS & AUTO-PAIRING MODULE
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
    bool isNewPeer = (!State::isPaired || memcmp(State::peerMAC, mac, 6) != 0);
    if (registerPeer(mac)) {
      memcpy(State::peerMAC, mac, 6);
      if (isNewPeer) {
        State::isPaired = true;
        State::pairedAtTime = millis();
        State::peerMacUpdated = true;
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
    outgoingPacket.magic = Config::MAGIC_ID;
    outgoingPacket.type = PKT_AUDIO;
    outgoingPacket.sequence = State::packetSequence++;
    memcpy(outgoingPacket.samples, samples, sizeof(outgoingPacket.samples));
    esp_now_send(State::peerMAC, (uint8_t*)&outgoingPacket, sizeof(outgoingPacket));
  }

  void onPacketSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {}

  // Fast, non-blocking ISR callback: pushes audio straight into ring buffer
  void onPacketReceived(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len != sizeof(WalkiePacket)) return;

    const WalkiePacket* incoming = reinterpret_cast<const WalkiePacket*>(data);
    if (incoming->magic != Config::MAGIC_ID) return;

    switch (incoming->type) {
      case PKT_DISCOVER:
        lockOntoPeer(info->src_addr);
        sendControlPacket(State::peerMAC, PKT_PAIR_ACK);
        break;

      case PKT_PAIR_ACK:
        lockOntoPeer(info->src_addr);
        break;

      case PKT_AUDIO:
        if (State::isPaired && memcmp(info->src_addr, State::peerMAC, 6) == 0 && !State::isTransmitting) {
          uint8_t nextHead = (State::rxHead + 1) % Config::JITTER_SLOTS;
          if (nextHead != State::rxTail) {
            memcpy(State::rxRing[State::rxHead], incoming->samples, sizeof(incoming->samples));
            State::rxHead = nextHead;
          }
          State::lastRxTime = millis();
        }
        break;
    }
  }

  void init() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(Config::WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
      while (true) delay(1000);
    }

    esp_now_register_send_cb(onPacketSent);
    esp_now_register_recv_cb(onPacketReceived);
    registerPeer(State::broadcastMAC);
  }
}

// ============================================================================
// SECTION 7: USER CONTROLS & RGB NEOPIXEL STATUS MODULE
// ============================================================================
namespace ControlsModule {
  uint8_t lastNeoState = 255;

  void setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b, uint8_t stateCode) {
    if (lastNeoState == stateCode) return;
    lastNeoState = stateCode;
    neopixelWrite(PIN_NEOPIXEL, r, g, b);
  }

  void init() {
    pinMode(Config::LED_PIN, OUTPUT);
    digitalWrite(Config::LED_PIN, LOW);

    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    setNeoPixelColor(25, 20, 0, 1);

    pinMode(Config::PTT_BUTTON_PIN, INPUT_PULLUP);
  }

  void blinkSearchHeartbeat() {
    digitalWrite(Config::LED_PIN, !digitalRead(Config::LED_PIN));
  }

  void update() {
    bool buttonHeld = (digitalRead(Config::PTT_BUTTON_PIN) == LOW);

    if (buttonHeld && !State::isTransmitting) {
      State::isTransmitting = true;
      digitalWrite(Config::LED_PIN, HIGH);
      AudioModule::muteSpeaker();
    } else if (!buttonHeld && State::isTransmitting) {
      State::isTransmitting = false;
      digitalWrite(Config::LED_PIN, LOW);
    }

    if (!State::isPaired) {
      setNeoPixelColor(25, 20, 0, 1);
    } else if (State::isTransmitting) {
      setNeoPixelColor(35, 10, 0, 2);
    } else if (millis() - State::lastRxTime < Config::RX_HOLD_TIMEOUT_MS) {
      setNeoPixelColor(0, 25, 30, 3);
    } else {
      setNeoPixelColor(0, 25, 0, 4);
    }
  }
}

// ============================================================================
// SECTION 8: MAIN APPLICATION ENTRY POINTS
// ============================================================================
void setup() {
  Serial.begin(115200);
  ControlsModule::init();
  BatteryModule::init();
  DisplayModule::init();
  AudioModule::init();
  WirelessModule::init();
}

void loop() {
  // 1. Drain jitter buffer to speaker FIRST so audio never waits on UI
  if (State::isPaired && !State::isTransmitting) {
    AudioModule::drainJitterBufferToSpeaker();
  }

  // 2. Update HUD & battery (pauses waveform redraws automatically during RX)
  BatteryModule::update();
  DisplayModule::update();

  // 3. Discovery pings if not yet paired
  if (!State::isPaired) {
    static unsigned long lastSearchPing = 0;
    if (millis() - lastSearchPing >= Config::DISCOVER_INTERVAL_MS) {
      lastSearchPing = millis();
      ControlsModule::blinkSearchHeartbeat();
      WirelessModule::sendControlPacket(State::broadcastMAC, PKT_DISCOVER);
    }
    return;
  }

  // 4. Poll PTT button & update NeoPixel
  ControlsModule::update();

  // 5. Capture & transmit mic frames when PTT is held
  if (State::isTransmitting) {
    int16_t micSamples[Config::AUDIO_SAMPLES];
    if (AudioModule::captureMicFrame(micSamples)) {
      WirelessModule::sendAudioFrame(micSamples);
      memcpy(State::waveSnapshot, micSamples, sizeof(State::waveSnapshot));
      State::newWaveAvailable = true;
    }
  }
}