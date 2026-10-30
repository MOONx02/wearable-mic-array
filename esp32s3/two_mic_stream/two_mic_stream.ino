// Two PDM mics on one data line (shared clock), streamed to the laptop over USB.
//   CLK  -> D1 (GPIO2), both mics
//   DATA -> D0 (GPIO1), both mics
// (The bench rig is wired this way round; mic_pin_finder confirms the wiring.)
//   SEL  -> GND on mic 1, 3V3 on mic 2 (each mic answers on a different clock edge)
// Packet: "M2" magic (0xA5 0x5A 'M' '2'), uint32 sequence, uint16 frame count,
// then frame count x 2 int16 samples, interleaved in the order the I2S driver returns them.
// Read it with tools/two_mic_capture.py.
//
// Uses the ESP-IDF PDM driver directly: the Arduino ESP_I2S wrapper fixes the PDM clock
// at 64 x fs (1.024 MHz at 16 kHz), the bottom edge of what the MP34DT01-M accepts.

#include "driver/i2s_pdm.h"

constexpr gpio_num_t PIN_PDM_CLK = GPIO_NUM_2;
constexpr gpio_num_t PIN_PDM_DATA = GPIO_NUM_1;
constexpr uint32_t SAMPLE_RATE = 16000;  // PDM clock = 128 x fs = 2.048 MHz
constexpr uint16_t FRAMES = 256;

i2s_chan_handle_t rx = nullptr;
int16_t samples[FRAMES * 2];
uint32_t seq = 0;

static void fail(const char *what, esp_err_t err) {
  while (true) {
    Serial.printf("ERROR: %s: %s\n", what, esp_err_to_name(err));
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  esp_err_t err = i2s_new_channel(&chan_cfg, nullptr, &rx);
  if (err != ESP_OK) fail("i2s_new_channel", err);

  i2s_pdm_rx_config_t cfg = {};
  cfg.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  cfg.clk_cfg.dn_sample_mode = I2S_PDM_DSR_16S;
  cfg.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  cfg.gpio_cfg.clk = PIN_PDM_CLK;
  cfg.gpio_cfg.din = PIN_PDM_DATA;
  err = i2s_channel_init_pdm_rx_mode(rx, &cfg);
  if (err != ESP_OK) fail("i2s_channel_init_pdm_rx_mode", err);
  err = i2s_channel_enable(rx);
  if (err != ESP_OK) fail("i2s_channel_enable", err);
}

void loop() {
  size_t got = 0;
  if (i2s_channel_read(rx, samples, sizeof(samples), &got, portMAX_DELAY) != ESP_OK) return;
  uint16_t frames = got / (2 * sizeof(int16_t));
  if (frames == 0) return;
  const uint8_t magic[4] = {0xA5, 0x5A, 'M', '2'};
  Serial.write(magic, sizeof(magic));
  Serial.write(reinterpret_cast<const uint8_t *>(&seq), sizeof(seq));
  Serial.write(reinterpret_cast<const uint8_t *>(&frames), sizeof(frames));
  Serial.write(reinterpret_cast<const uint8_t *>(samples), frames * 2 * sizeof(int16_t));
  seq++;
}
