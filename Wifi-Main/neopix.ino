// NeoPixel brightness, 0 (min) to 255 (max)
uint32_t LED_BRIGHTNESS = 150;

void setupLed() {
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(LED_BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void setStrip(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM but we shoudln't miss this memory too much)
  }
  strip.show();                          //  Update strip to match
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
