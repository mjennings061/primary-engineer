// NeoPixel brightness, 0 (min) to 255 (max)
uint32_t LED_BRIGHTNESS = 255;

void setupLed() {
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(LED_BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void setStrip(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
  }
  strip.show();                          //  Update strip to match
}

void drawX(uint32_t color) {
  strip.setBrightness(LED_BRIGHTNESS);
  strip.setPixelColor(0,color);
  strip.setPixelColor(1,black);
  strip.setPixelColor(2,color);
  strip.setPixelColor(3,color);
  strip.setPixelColor(4,black);
  strip.setPixelColor(5,color);
  strip.setPixelColor(6,color);
  strip.show();
  strip.setBrightness(LED_BRIGHTNESS);
}

void drawDash(uint32_t color) {
  //strip.setBrightness(255);
  strip.setPixelColor(0,color);
  strip.setPixelColor(1,color);
  strip.setPixelColor(2,black);
  strip.setPixelColor(3,black);
  strip.setPixelColor(4,color);
  strip.setPixelColor(5,black);
  strip.setPixelColor(6,black);
  strip.show();
  //strip.setBrightness(LED_BRIGHTNESS);
}

void testLed() {
  setStrip(red);
  delay(250);
  setStrip(green);
  delay(250);
  setStrip(blue);
  delay(250);
  setStrip(amber);
  delay(250);
  setStrip(aqua);
  delay(250);
  setStrip(purple);
  delay(250);
  setStrip(white);
  delay(250);
  setStrip(black);
}
