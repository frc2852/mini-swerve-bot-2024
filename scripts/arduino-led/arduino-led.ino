#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define LED_PIN 6   // The pin your LED strip is connected to
#define NUM_LEDS 50 // Number of LEDs in your strip
#define I2C_ADDRESS 0x10 // I2C address of your Arduino

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // I2C communication is handled in the receiveEvent function
  delay(100); // Small delay to prevent overwhelming the MCU
}

void receiveEvent(int howMany) {
  if (Wire.available() > 0) {
    String rgbString = "";
    while (Wire.available()) {
      char c = Wire.read();
      rgbString += c;
    }
    int r, g, b;
    parseRGBString(rgbString, r, g, b);
    setAllLEDs(strip.Color(r, g, b)); // Set the color of the LEDs
    strip.show();
  }
}

void parseRGBString(String rgbString, int &r, int &g, int &b) {
  int firstCommaIndex = rgbString.indexOf(',');
  int secondCommaIndex = rgbString.indexOf(',', firstCommaIndex + 1);

  r = rgbString.substring(0, firstCommaIndex).toInt();
  g = rgbString.substring(firstCommaIndex + 1, secondCommaIndex).toInt();
  b = rgbString.substring(secondCommaIndex + 1).toInt();
}

void setAllLEDs(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
}
