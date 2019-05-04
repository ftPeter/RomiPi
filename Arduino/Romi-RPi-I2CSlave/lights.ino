/*
   Controlling neopixel lights for visual status notification

   based on the Neopixel example from the adafruit neopixel library
*/

#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN  1
#define NUMPIXELS     2

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void lights_init() {
  pixels.begin(); // This initializes the NeoPixel library.
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(0, pixels.Color(i, i, i));
    pixels.show();
    delay(10);
  }
}

void lights_update(int8_t red, int8_t green, int8_t blue) {
  // update pixel colors
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(slave.buffer.pixel_red,
                                         slave.buffer.pixel_green,
                                         slave.buffer.pixel_blue));
  }
  pixels.show();
}

