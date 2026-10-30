// Built-in mic check for the XIAO ESP32-S3 Sense.
// Reads the on-board PDM mic (CLK GPIO42, DATA GPIO41) at 16 kHz and prints
// the loudness of every 100 ms block. Talk or clap: the level should jump.

#include <ESP_I2S.h>

constexpr int PIN_PDM_CLK = 42;
constexpr int PIN_PDM_DATA = 41;
constexpr uint32_t SAMPLE_RATE = 16000;
constexpr size_t BLOCK = SAMPLE_RATE / 10;

I2SClass mic;
int16_t samples[BLOCK];

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nXIAO ESP32-S3 Sense built-in mic test");

  mic.setPinsPdmRx(PIN_PDM_CLK, PIN_PDM_DATA);
  if (!mic.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("ERROR: PDM receive failed to start");
    while (true) delay(1000);
  }
  // The mic needs a moment after its clock starts before the data settles.
  for (int i = 0; i < 5; i++) mic.readBytes(reinterpret_cast<char *>(samples), sizeof(samples));
  Serial.printf("PDM receive running: %lu Hz, 16-bit, mono. PSRAM: %u bytes\n",
                static_cast<unsigned long>(SAMPLE_RATE), static_cast<unsigned>(ESP.getPsramSize()));
}

void loop() {
  size_t got = mic.readBytes(reinterpret_cast<char *>(samples), sizeof(samples)) / sizeof(int16_t);
  if (got == 0) {
    Serial.println("ERROR: no samples");
    delay(500);
    return;
  }

  double mean = 0;
  for (size_t i = 0; i < got; i++) mean += samples[i];
  mean /= got;

  double power = 0;
  int peak = 0;
  for (size_t i = 0; i < got; i++) {
    double v = samples[i] - mean;
    power += v * v;
    peak = max(peak, abs(static_cast<int>(v)));
  }
  double dbfs = 10.0 * log10(power / got / (32768.0 * 32768.0) + 1e-12);

  int bars = constrain(static_cast<int>((dbfs + 80.0) / 2.0), 0, 40);
  char meter[41];
  for (int i = 0; i < 40; i++) meter[i] = i < bars ? '#' : ' ';
  meter[40] = '\0';
  Serial.printf("level %6.1f dBFS  peak %5d  dc %7.1f  |%s|\n", dbfs, peak, mean, meter);
}
