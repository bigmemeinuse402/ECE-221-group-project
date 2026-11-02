
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// Display pins
#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 6

// Push button
#define BUTTON_PIN A0

Adafruit_ST7735 tft =
  Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

bool previousState = false;
bool firstUpdate = true;

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize display
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_MAGENTA);

}

void loop() {
  
  bool pressed = (digitalRead(BUTTON_PIN) == LOW);
  //Serial.println(digitalRead(BUTTON_PIN));
  if (firstUpdate || pressed != previousState) {

    tft.fillScreen(ST77XX_MAGENTA);

    if (pressed) {
      tft.setTextColor(ST77XX_GREEN);
      tft.setTextSize(2);
      tft.setCursor(20, 55);
      tft.println("READY FOR\nTRANSMISSION");

      //tft.setTextSize(1);
      //tft.setCursor(20, 55);
      //tft.println("FOR TRANSMISSION");

      Serial.println("READY FOR TRANSMISSION");

      //code here
    }
    else {
      tft.setTextColor(ST77XX_BLUE);
      tft.setTextSize(2);
      tft.setCursor(35, 40);
      tft.println("STANDBY");

      Serial.println("STANDBY");
    }

    previousState = pressed;
    firstUpdate = false;
  }

  delay(50);
}
