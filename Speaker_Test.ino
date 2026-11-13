
#include <Arduino.h>
#include <ESP_I2S.h>
#include <math.h>

// MAX98357A amplifier connections
#define SPK_BCLK 12
#define SPK_LRC  13
#define SPK_DIN  10

#define SAMPLE_RATE 16000
#define TONE_FREQ 440

I2SClass speaker;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("Starting speaker test...");

  speaker.setPins(SPK_BCLK, SPK_LRC, SPK_DIN);

  if (!speaker.begin(
      I2S_MODE_STD,
      SAMPLE_RATE,
      I2S_DATA_BIT_WIDTH_16BIT,
      I2S_SLOT_MODE_STEREO)) {
    Serial.println("Speaker setup failed!");
    while (true) delay(1000);
  }

  Serial.println("Speaker ready!");
}

void loop() {
  const int samples = 160;
  int16_t buffer[samples * 2];

  // Play a 440 Hz tone for 0.3 seconds
  for (int block = 0; block < 30; block++) {
    for (int i = 0; i < samples; i++) {
      int sampleNumber = block * samples + i;
      float t = (float)sampleNumber / SAMPLE_RATE;

      int16_t value = (int16_t)(
        5000 * sinf(2.0f * PI * TONE_FREQ * t)
      );

      buffer[i * 2] = value;
      buffer[i * 2 + 1] = value;
    }

    speaker.write(
      (uint8_t*)buffer,
      sizeof(buffer)
    );
  }

  delay(1700);
}
