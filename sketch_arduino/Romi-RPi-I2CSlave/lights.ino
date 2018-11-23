/*
   Controlling neopixel lights for visual status notification

   based on the Neopixel example from the adafruit neopixel library
*/

// NeoPixel Setup Materials
// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// Which pin on the Romi is connected to the NeoPixels?
#define PIN            1

// How many NeoPixels are attached to the Romi?
#define NUMPIXELS      2

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// end NeoPixel Setup


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

