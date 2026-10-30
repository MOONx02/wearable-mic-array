// Finds where the PDM mics are actually wired on the XIAO ESP32-S3.
// 1. Reads every D pin with pull-up and pull-down to find pins tied to 3V3 or GND
//    (those are never driven by this sketch).
// 2. For each remaining pin as a candidate clock: drives it high and low to find pins
//    wired directly to it, then runs a 2 MHz clock and lists pins that become driven
//    (a mic's DAT output answers only while its clock runs).
// Repeats every few seconds, so you can rewire while it runs.

#include "driver/gpio.h"

struct Pin {
  const char *name;
  int gpio;
};
const Pin PINS[] = {{"D0", 1}, {"D1", 2}, {"D2", 3}, {"D3", 4}, {"D4", 5}, {"D5", 6},
                    {"D6", 43}, {"D7", 44}, {"D8", 7}, {"D9", 8}, {"D10", 9}};
constexpr int N = sizeof(PINS) / sizeof(PINS[0]);

static float ones(int gpio, gpio_pull_mode_t pull, int n = 20000) {
  gpio_num_t g = static_cast<gpio_num_t>(gpio);
  gpio_reset_pin(g);
  gpio_set_direction(g, GPIO_MODE_INPUT);
  gpio_set_pull_mode(g, pull);
  delayMicroseconds(300);
  int c = 0;
  for (int i = 0; i < n; i++) c += gpio_get_level(g);
  return 100.0f * c / n;
}

// 'G' tied to GND, 'V' tied to 3V3, 'D' driven by something, 'F' floating
static char state_of(int gpio) {
  float up = ones(gpio, GPIO_PULLUP_ONLY), down = ones(gpio, GPIO_PULLDOWN_ONLY);
  if (up < 2 && down < 2) return 'G';
  if (up > 98 && down > 98) return 'V';
  if (up - down < 30) return 'D';
  return 'F';
}

void setup() {
  Serial.begin(115200);
  delay(1500);
}

void loop() {
  Serial.println("\n=== pin finder ===");
  char st[N];
  Serial.print("Idle:   ");
  for (int i = 0; i < N; i++) {
    st[i] = state_of(PINS[i].gpio);
    const char *txt = st[i] == 'G' ? "GND" : st[i] == 'V' ? "3V3" : st[i] == 'D' ? "driven" : "-";
    Serial.printf("%s=%s  ", PINS[i].name, txt);
  }
  Serial.println();

  bool any = false;
  for (int c = 0; c < N; c++) {
    if (st[c] == 'G' || st[c] == 'V') continue;
    gpio_num_t cg = static_cast<gpio_num_t>(PINS[c].gpio);
    String wired, data;
    // direct connections: other pins that follow a static high/low on pin c
    gpio_reset_pin(cg);
    gpio_set_direction(cg, GPIO_MODE_OUTPUT);
    for (int o = 0; o < N; o++) {
      if (o == c || st[o] == 'G' || st[o] == 'V') continue;
      gpio_set_level(cg, 1);
      float hi = ones(PINS[o].gpio, GPIO_PULLDOWN_ONLY, 2000);
      gpio_set_level(cg, 0);
      float lo = ones(PINS[o].gpio, GPIO_PULLUP_ONLY, 2000);
      if (hi > 95 && lo < 5) wired += String(PINS[o].name) + " ";
    }
    gpio_reset_pin(cg);
    // mic data: pins that become driven only while a clock runs on pin c
    ledcAttach(PINS[c].gpio, 2000000, 4);
    ledcWrite(PINS[c].gpio, 8);
    delay(30);
    for (int o = 0; o < N; o++) {
      if (o == c || st[o] != 'F' || wired.indexOf(String(PINS[o].name) + " ") >= 0) continue;
      float up = ones(PINS[o].gpio, GPIO_PULLUP_ONLY), down = ones(PINS[o].gpio, GPIO_PULLDOWN_ONLY);
      if (up - down < 30) data += String(PINS[o].name) + "(" + String(up, 0) + "%) ";
    }
    ledcDetach(PINS[c].gpio);
    gpio_reset_pin(cg);
    if (wired.length() || data.length()) {
      any = true;
      Serial.printf("Clock on %-3s -> wired directly to: %s | mic data appears on: %s\n", PINS[c].name,
                    wired.length() ? wired.c_str() : "none", data.length() ? data.c_str() : "none");
    }
  }
  if (!any) Serial.println("No clock pin wakes up any mic, and no two D pins are wired together.");
  delay(1500);
}
