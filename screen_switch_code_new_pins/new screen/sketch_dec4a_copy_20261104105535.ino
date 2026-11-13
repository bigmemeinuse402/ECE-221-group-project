
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

//display pins
#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 6

//button
#define BUTTON_PIN A0

Adafruit_ST7735 tft =
  Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

bool previousState = false;
bool firstUpdate = true;


void showStartup() {
  tft.fillScreen(ST77XX_MAGENTA);

  //display name
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 35);
  tft.println("flushphone");

  //white heart
  tft.fillCircle(70, 85, 10, ST77XX_WHITE);
  tft.fillCircle(90, 85, 10, ST77XX_WHITE);
  tft.fillTriangle(60, 89, 100, 89, 80, 115,ST77XX_WHITE);
}



void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  //initialize 
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);

  //startup screen for 3 sec
  showStartup();
  delay(3000);

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

      //Serial.println("READY FOR TRANSMISSION");

      //code here
    }
    else {
      tft.setTextColor(ST77XX_BLUE);
      tft.setTextSize(2);
      tft.setCursor(35, 40);
      tft.println("STANDBY");

     
    }

    previousState = pressed;
    firstUpdate = false;
  }

  delay(50);
}
