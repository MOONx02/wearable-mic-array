// PWM audio test: plays test sounds out of D10 (GPIO9), no DAC.
//
// Hookup (pick one), never wire a bare 8 ohm speaker straight to the pin:
//   Powered speaker / AUX in:  D10 -> 1k -> node -> 47nF -> GND
//                              node -> 1uF -> 3.5mm tip (+ ring), sleeve -> GND
//   Headphones (32 ohm):       D10 -> 100R -> 10uF (+ side to pin) -> tip, sleeve -> GND
//   Bare speaker:              filtered node -> small amp module (PAM8302 / LM386) -> speaker
//
// Serial commands (115200): d t s m a x + - ?

#include <Arduino.h>

const int OUT_PIN = D10;
const uint32_t PWM_FREQ = 78125;   // 80 MHz / 1024
const uint8_t PWM_BITS = 10;       // duty 0..1023
const int MID = 512;               // silence = half duty = ~1.65 V after filter
const uint32_t SAMPLE_US = 62;
const float FS = 1000000.0f / SAMPLE_US;   // ~16.1 kHz

const float MELODY[] = {523.3f, 587.3f, 659.3f, 784.0f, 659.3f, 587.3f, 523.3f, 0};
const int MELODY_LEN = sizeof(MELODY) / sizeof(MELODY[0]);

char mode = 'd';
int amp = 200;          // out of 511; keep low for headphones
float phase = 0;
uint32_t n = 0;         // samples since mode started

void help() {
  Serial.println();
  Serial.println("PWM speaker test on D10");
  Serial.println("  d  DC level: 25% duty (measure ~0.83 V after the filter)");
  Serial.println("  t  1 kHz tone");
  Serial.println("  s  sweep 200 -> 3000 Hz");
  Serial.println("  m  melody");
  Serial.println("  a  alert beeps");
  Serial.println("  x  silence");
  Serial.println("  + / -  volume");
  Serial.printf("mode=%c volume=%d/511\n", mode, amp);
}

float tone(float hz) {
  phase += TWO_PI * hz / FS;
  if (phase > TWO_PI) phase -= TWO_PI;
  return sinf(phase);
}

int nextSample() {
  float t = n / FS;
  switch (mode) {
    case 'd':
      return 256;
    case 't':
      return MID + (int)(amp * tone(1000));
    case 's': {
      float hz = 200 + 2800 * fmodf(t, 3.0f) / 3.0f;
      return MID + (int)(amp * tone(hz));
    }
    case 'm': {
      int note = (int)(t / 0.3f) % MELODY_LEN;
      bool gap = fmodf(t, 0.3f) > 0.25f;
      if (MELODY[note] == 0 || gap) return MID;
      return MID + (int)(amp * tone(MELODY[note]));
    }
    case 'a': {
      float c = fmodf(t, 1.0f);
      bool on = c < 0.15f || (c > 0.25f && c < 0.40f);
      return on ? MID + (int)(amp * tone(2000)) : MID;
    }
    default:
      return MID;
  }
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c && strchr("dtsmax", c)) {
    mode = c;
    n = 0;
    phase = 0;
  } else if (c == '+') {
    amp = min(amp + 50, 500);
  } else if (c == '-') {
    amp = max(amp - 50, 0);
  } else if (c != '?') {
    return;
  }
  Serial.printf("mode=%c volume=%d/511\n", mode, amp);
  if (c == '?') help();
}

void setup() {
  Serial.begin(115200);
  ledcAttach(OUT_PIN, PWM_FREQ, PWM_BITS);
  ledcWrite(OUT_PIN, MID);
  delay(1500);
  help();
}

void loop() {
  static uint32_t next = micros();
  while ((int32_t)(micros() - next) < 0) {}
  next += SAMPLE_US;

  ledcWrite(OUT_PIN, constrain(nextSample(), 0, 1023));
  n++;

  if ((n & 255) == 0) handleSerial();
}
