// Vest firmware skeleton: one FreeRTOS task per job, audio handed between tasks through queues.
//   micTask    core 1, priority 5   reads the PDM mics, sends each block to the voice and hazard queues
//   voiceTask  core 1, priority 4   combines the mics while push-to-talk is held (beam + noise suppression go here)
//   hazardTask core 0, priority 3   flags loud events with a timestamp
//   pttTask    core 0, priority 2   push-to-talk button -> radio PTT optocoupler
//   logTask    core 0, priority 1   prints status once a second (later: events to the laptop over Wi-Fi)
// Bench wiring as two_mic_stream: CLK -> D1, DATA -> D0. PTT button D6 -> GND, optocoupler on D7.
// Prints text, so the live viewer and two_mic_capture.py won't read it.

#include "driver/i2s_pdm.h"

constexpr gpio_num_t PIN_PDM_CLK = GPIO_NUM_2;
constexpr gpio_num_t PIN_PDM_DATA = GPIO_NUM_1;
constexpr int PIN_PTT_BUTTON = D6;
constexpr int PIN_PTT_RADIO = D7;
constexpr uint32_t SAMPLE_RATE = 16000;
constexpr int MICS = 2;
constexpr int FRAMES = 256;               // 16 ms of audio per block
constexpr float EVENT_DB = -20.0f;        // loud-event threshold, dBFS
constexpr uint32_t EVENT_HOLDOFF_MS = 500;
constexpr uint32_t RADIO_KEYUP_MS = 150;  // radio needs this long after PTT before it transmits

struct Block {
  uint32_t t_ms;
  int16_t s[FRAMES * MICS];  // interleaved: mic0, mic1, mic0, mic1, ...
};

struct Event {
  uint32_t t_ms;
  float db;
};

i2s_chan_handle_t rx = nullptr;
QueueHandle_t voiceQ, hazardQ, eventQ;

volatile bool talking = false;
volatile uint32_t blocksIn = 0, blocksDropped = 0;
volatile float voiceDb = -99.0f;

float levelDb(const int16_t *s, int stride, int n) {
  float mean = 0;
  for (int i = 0; i < n; i++) mean += s[i * stride];
  mean /= n;
  float sum = 0;
  for (int i = 0; i < n; i++) {
    float v = s[i * stride] - mean;
    sum += v * v;
  }
  return 10.0f * log10f(sum / n / (32768.0f * 32768.0f) + 1e-12f);
}

void micTask(void *) {
  Block b;
  for (;;) {
    size_t got = 0;
    i2s_channel_read(rx, b.s, sizeof(b.s), &got, portMAX_DELAY);  // sleeps until the DMA has a block
    b.t_ms = millis();
    blocksIn++;
    if (xQueueSend(voiceQ, &b, 0) != pdTRUE) blocksDropped++;
    if (xQueueSend(hazardQ, &b, 0) != pdTRUE) blocksDropped++;
  }
}

void voiceTask(void *) {
  Block b;
  int16_t mono[FRAMES];
  for (;;) {
    xQueueReceive(voiceQ, &b, portMAX_DELAY);
    if (!talking) continue;
    for (int i = 0; i < FRAMES; i++) mono[i] = (b.s[i * MICS] + b.s[i * MICS + 1]) / 2;
    // Beam + noise suppression go here, then mono[] goes out to the radio's mic line.
    voiceDb = levelDb(mono, 1, FRAMES);
  }
}

void hazardTask(void *) {
  Block b;
  uint32_t lastEvent = 0;
  for (;;) {
    xQueueReceive(hazardQ, &b, portMAX_DELAY);
    float db = max(levelDb(b.s, MICS, FRAMES), levelDb(b.s + 1, MICS, FRAMES));
    if (db > EVENT_DB && b.t_ms - lastEvent > EVENT_HOLDOFF_MS) {
      lastEvent = b.t_ms;
      Event e = {b.t_ms, db};
      xQueueSend(eventQ, &e, 0);
    }
  }
}

void pttTask(void *) {
  bool held = false;
  for (;;) {
    bool pressed = digitalRead(PIN_PTT_BUTTON) == LOW;
    if (pressed != held) {
      vTaskDelay(pdMS_TO_TICKS(20));  // debounce
      if ((digitalRead(PIN_PTT_BUTTON) == LOW) == pressed) {
        held = pressed;
        digitalWrite(PIN_PTT_RADIO, pressed);
        if (pressed) vTaskDelay(pdMS_TO_TICKS(RADIO_KEYUP_MS));
        talking = pressed;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void logTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
    Event e;
    while (xQueueReceive(eventQ, &e, 0) == pdTRUE) {
      Serial.printf("EVENT  t=%lu ms  %.1f dBFS\n", (unsigned long)e.t_ms, e.db);
    }
    Serial.printf("t=%lus  blocks %lu  dropped %lu  ptt %s  voice %.1f dBFS  free heap %lu\n",
                  (unsigned long)(millis() / 1000), (unsigned long)blocksIn, (unsigned long)blocksDropped,
                  talking ? "ON " : "off", voiceDb, (unsigned long)ESP.getFreeHeap());
  }
}

void startMics() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx));
  i2s_pdm_rx_config_t cfg = {};
  cfg.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  cfg.clk_cfg.dn_sample_mode = I2S_PDM_DSR_16S;
  cfg.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  cfg.gpio_cfg.clk = PIN_PDM_CLK;
  cfg.gpio_cfg.din = PIN_PDM_DATA;
  ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx, &cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PTT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_PTT_RADIO, OUTPUT);
  digitalWrite(PIN_PTT_RADIO, LOW);
  startMics();

  voiceQ = xQueueCreate(8, sizeof(Block));  // 8 blocks = 128 ms of slack
  hazardQ = xQueueCreate(8, sizeof(Block));
  eventQ = xQueueCreate(16, sizeof(Event));

  xTaskCreatePinnedToCore(micTask, "mic", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(voiceTask, "voice", 4096, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(hazardTask, "hazard", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(pttTask, "ptt", 2048, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(logTask, "log", 4096, nullptr, 1, nullptr, 0);
}

void loop() {
  vTaskDelete(nullptr);  // everything runs in the tasks above; free loopTask's stack
}
