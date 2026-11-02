/*
 * ESP32-S3 + standard Philips I2S microphone input test.
 * Arduino-ESP32 3.x, ESP_I2S (included with the board package).
 * Assumes an INMP441-compatible, left-selected, MSB-aligned microphone.
 * The exact microphone chip has not yet been confirmed from its photo.
 *
 * Wiring: VDD -> 3V3, GND -> GND, SCK -> GPIO4,
 *         WS -> GPIO5, SD -> GPIO6, L/R -> GND.
 * Serial Plotter: 115200 baud. Prints RMS and peak-to-peak every 100 ms.
 * Output PCM: 16 kHz, mono, signed 16-bit, little endian on ESP32-S3.
 * Optional: capture_mic.py requests a three-second recording with 'r'.
 * I2S initialization does NOT check whether a microphone is connected.
 */

#include <Arduino.h>
#include <ESP_I2S.h>
#include <math.h>
#include <esp_arduino_version.h>
#include <driver/gpio.h>

#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "This sketch requires the esp32 by Espressif Systems 3.x board package."
#endif
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select ESP32S3 Dev Module (or the correct ESP32-S3 board)."
#endif

constexpr int MIC_BCLK = 4;
constexpr int MIC_WS = 5;
constexpr int MIC_DATA = 6;
constexpr uint32_t SAMPLE_RATE = 16000;
constexpr size_t BLOCK_SAMPLES = SAMPLE_RATE / 100;  // 10 ms = 160 samples
constexpr uint8_t MIC_CHANNEL = 0;                  // Left, L/R wired to GND
constexpr uint32_t RECORD_SECONDS = 3;
constexpr bool PRINT_LEVELS = true;

I2SClass microphone;
int32_t rawStereo[BLOCK_SAMPLES * 2];
int16_t pcmBuffer[BLOCK_SAMPLES];

// Return the valid MONO sample count, not the byte count.
// dest must hold at least BLOCK_SAMPLES int16_t values.
size_t readMicPCM(int16_t *dest) {
  const size_t bytesRead = microphone.readBytes(
      reinterpret_cast<char *>(rawStereo), sizeof(rawStereo));
  const size_t frameBytes = 2 * sizeof(int32_t);
  if (bytesRead == 0 || bytesRead % frameBytes != 0) {
    return 0;
  }

  const size_t samplesRead = bytesRead / frameBytes;
  for (size_t i = 0; i < samplesRead; ++i) {
    // Read [L,R,L,R,...]; keep the left channel and its signed top 16 bits.
    // INMP441 supplies 24 valid bits within each 32-bit I2S slot.
    dest[i] = static_cast<int16_t>(rawStereo[2 * i + MIC_CHANNEL] >> 16);
  }
  return samplesRead;
}

// Integration point for the teammate's sender.
// This buffer is reused on the next read: consume it now or copy it to a queue.
// Keep this handler short; avoid long delays or waiting for radio retries here.
void onAudioBlock(const int16_t *samples, size_t count) {
  if (!PRINT_LEVELS) return;
  static int64_t sum = 0;
  static uint64_t sumSquares = 0;
  static size_t windowSamples = 0;
  static int32_t minimum = 32767;
  static int32_t maximum = -32768;
  static uint32_t lastPrint = 0;

  for (size_t i = 0; i < count; ++i) {
    const int32_t value = samples[i];
    sum += value;
    sumSquares += static_cast<uint64_t>(static_cast<int64_t>(value) * value);
    if (value < minimum) minimum = value;
    if (value > maximum) maximum = value;
  }
  windowSamples += count;

  const uint32_t now = millis();
  if (now - lastPrint >= 100 && windowSamples > 0) {
    const double mean = static_cast<double>(sum) / windowSamples;
    double variance = static_cast<double>(sumSquares) / windowSamples - mean * mean;
    if (variance < 0) variance = 0;

    Serial.print("RMS:");
    Serial.print(sqrt(variance), 1);
    Serial.print("\tPeakToPeak:");
    Serial.println(maximum - minimum);

    sum = 0;
    sumSquares = 0;
    windowSamples = 0;
    minimum = 32767;
    maximum = -32768;
    lastPrint = now;
  }
}

void writeLE16(uint16_t value) {
  const uint8_t bytes[] = {static_cast<uint8_t>(value),
                           static_cast<uint8_t>(value >> 8)};
  Serial.write(bytes, sizeof(bytes));
}

void writeLE32(uint32_t value) {
  const uint8_t bytes[] = {static_cast<uint8_t>(value),
                           static_cast<uint8_t>(value >> 8),
                           static_cast<uint8_t>(value >> 16),
                           static_cast<uint8_t>(value >> 24)};
  Serial.write(bytes, sizeof(bytes));
}

// Record first, then export: 115200-baud UART cannot stream 16-kHz PCM live.
// Use capture_mic.py; a Serial Monitor would display the WAV as binary noise.
void recordAndExportWAV() {
  const size_t targetSamples = SAMPLE_RATE * RECORD_SECONDS;
  int16_t *recording = static_cast<int16_t *>(malloc(targetSamples * sizeof(int16_t)));
  if (recording == nullptr) {
    Serial.println("RECORD_ERROR: not enough memory");
    return;
  }

  Serial.println("RECORDING: speak for 3 seconds now");
  size_t recorded = 0;
  while (recorded < targetSamples) {
    const size_t count = readMicPCM(pcmBuffer);
    if (count == 0) {
      free(recording);
      Serial.println("RECORD_ERROR: incomplete I2S read");
      return;
    }
    const size_t remaining = targetSamples - recorded;
    const size_t toCopy = count < remaining ? count : remaining;
    memcpy(recording + recorded, pcmBuffer, toCopy * sizeof(int16_t));
    recorded += toCopy;
  }

  const uint32_t dataBytes = recorded * sizeof(int16_t);
  Serial.print("WAV_BEGIN ");
  Serial.println(44 + dataBytes);
  Serial.write(reinterpret_cast<const uint8_t *>("RIFF"), 4);
  writeLE32(36 + dataBytes);
  Serial.write(reinterpret_cast<const uint8_t *>("WAVEfmt "), 8);
  writeLE32(16);                   // PCM format chunk size
  writeLE16(1);                    // Linear PCM
  writeLE16(1);                    // Mono
  writeLE32(SAMPLE_RATE);
  writeLE32(SAMPLE_RATE * 2);       // Bytes per second
  writeLE16(2);                    // Block alignment
  writeLE16(16);                   // Bits per sample
  Serial.write(reinterpret_cast<const uint8_t *>("data"), 4);
  writeLE32(dataBytes);
  Serial.write(reinterpret_cast<const uint8_t *>(recording), dataBytes);
  Serial.flush();
  Serial.println();
  Serial.println("WAV_END");
  free(recording);

  // Discard data accumulated by DMA during the slow serial export.
  for (int i = 0; i < 20; ++i) readMicPCM(pcmBuffer);
}

void setup() {
  Serial.begin(115200);
  delay(800);  // No indefinite wait for a connected Serial Monitor.
  Serial.println("ESP32_OK: program started");

  // Both 32-bit slots remain on the wire (64 clocks/frame).
  // This is microphone input only: DOUT and MCLK are unused (-1).
  microphone.setPins(MIC_BCLK, MIC_WS, -1, MIC_DATA, -1);
  microphone.setTimeout(200);
  if (!microphone.begin(I2S_MODE_STD, SAMPLE_RATE,
                        I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println("ERROR: I2S initialization failed.");
    while (true) delay(1000);
  }
  // Keep an unconnected data input from freely floating. This still does not
  // make I2S an addressable bus or provide a microphone-presence check.
  gpio_set_pull_mode(static_cast<gpio_num_t>(MIC_DATA), GPIO_PULLDOWN_ONLY);
  Serial.println("I2S_READY: verify sound response; this is not a presence check");

  // Discard startup audio while the microphone settles.
  for (int i = 0; i < 20; ++i) readMicPCM(pcmBuffer);
}

void loop() {
  if (Serial.available()) {
    const int command = Serial.read();
    if (command == 'r' || command == 'R') recordAndExportWAV();
  }
  const size_t count = readMicPCM(pcmBuffer);
  if (count == 0) {
    Serial.println("ERROR: no complete I2S audio frames; check wiring/settings.");
    delay(100);
    return;
  }
  onAudioBlock(pcmBuffer, count);
}
