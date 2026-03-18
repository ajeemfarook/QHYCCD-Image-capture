#include <Adafruit_NeoPixel.h>

// Pin and LED count
#define PIN_BIG    6
#define LEDS_BIG   24

#define PIN_SMALL  5
#define LEDS_SMALL 8

// Create NeoPixel objects
Adafruit_NeoPixel ringBig(LEDS_BIG, PIN_BIG, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ringSmall(LEDS_SMALL, PIN_SMALL, NEO_GRB + NEO_KHZ800);

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);   // Reliable baud rate for Python

  ringBig.begin();
  ringSmall.begin();

  ringBig.show();       // All OFF
  ringSmall.show();     // All OFF
}

// -------------------- LOOP --------------------
void loop() {
  // Small ring chase
  chaseRing(ringSmall, 0, 255, 0, 1000);
  setAll(ringSmall, 0, 0, 0);

  // Big ring chase
  chaseRing(ringBig, 0, 255, 0, 1000);
  setAll(ringBig, 0, 0, 0);

  delay(1000);
}

// -------------------- FUNCTIONS --------------------

// Send trigger to Python
void sendTrigger() {
  Serial.println("T");   // Send trigger with newline
}

// Chase one LED at a time and send trigger
void chaseRing(Adafruit_NeoPixel &ring, int r, int g, int b, int wait) {
  for (int i = 0; i < ring.numPixels(); i++) {
    ring.clear();
    ring.setPixelColor(i, ring.Color(r, g, b));
    ring.show();

    sendTrigger();       // 🔔 Trigger for Python

    delay(wait);
  }
}

// Set all LEDs in a ring to one color
void setAll(Adafruit_NeoPixel &ring, int r, int g, int b) {
  for (int i = 0; i < ring.numPixels(); i++) {
    ring.setPixelColor(i, ring.Color(r, g, b));
  }
  ring.show();
}
