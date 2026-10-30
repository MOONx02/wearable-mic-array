// Wiring check for the external PDM mics, plus a control test on the built-in mic.
// 1. Drives a 2 MHz clock on D0 (GPIO1) and samples D1 (GPIO2) with the internal
//    pull-up, then the pull-down. Working mics drive the line, so the share of 1s
//    barely changes; a floating line follows the pull (~100% then ~0%).
// 2. Runs the IDF PDM driver on the built-in mic (CLK GPIO42, DATA GPIO41) and
//    prints its level, to show the driver code itself works.
// Repeats every few seconds.

#include "driver/gpio.h"
#include "driver/i2s_pdm.h"

constexpr int EXT_CLK = 1, EXT_DATA = 2;
constexpr gpio_num_t INT_CLK = GPIO_NUM_42, INT_DATA = GPIO_NUM_41;

static float ones_share(gpio_pull_mode_t pull) {
  gpio_set_direction(static_cast<gpio_num_t>(EXT_DATA), GPIO_MODE_INPUT);
  gpio_set_pull_mode(static_cast<gpio_num_t>(EXT_DATA), pull);
  delay(20);
  uint32_t ones = 0;
  const uint32_t n = 200000;
  for (uint32_t i = 0; i < n; i++) ones += gpio_get_level(static_cast<gpio_num_t>(EXT_DATA));
  return 100.0f * ones / n;
}

static void external_check() {
  ledcAttach(EXT_CLK, 2000000, 4);
  ledcWrite(EXT_CLK, 8);
  delay(50);  // mics wake up within ~10 ms of the clock starting
  float up = ones_share(GPIO_PULLUP_ONLY);
  float down = ones_share(GPIO_PULLDOWN_ONLY);
  ledcDetach(EXT_CLK);
  delay(20);
  float up_noclk = ones_share(GPIO_PULLUP_ONLY);
  float down_noclk = ones_share(GPIO_PULLDOWN_ONLY);
  gpio_set_pull_mode(static_cast<gpio_num_t>(EXT_DATA), GPIO_FLOATING);
  Serial.printf("External mics, D1 share of 1s  with clock: pull-up %5.1f%%  pull-down %5.1f%%   "
                "| without clock: pull-up %5.1f%%  pull-down %5.1f%%\n", up, down, up_noclk, down_noclk);
  bool driven = (up - down) < 30.0f;
  Serial.printf("  -> %s\n", driven ? "D1 is being driven (mic data present)"
                                    : "D1 is floating: no mic is driving the data line");
}

static void builtin_check() {
  i2s_chan_handle_t rx = nullptr;
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  if (i2s_new_channel(&chan_cfg, nullptr, &rx) != ESP_OK) {
    Serial.println("Built-in mic: i2s_new_channel failed");
    return;
  }
  i2s_pdm_rx_config_t cfg = {};
  cfg.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(16000);
  cfg.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  cfg.gpio_cfg.clk = INT_CLK;
  cfg.gpio_cfg.din = INT_DATA;
  i2s_channel_init_pdm_rx_mode(rx, &cfg);
  i2s_channel_enable(rx);
  static int16_t buf[1600];
  size_t got = 0;
  for (int i = 0; i < 4; i++) i2s_channel_read(rx, buf, sizeof(buf), &got, 1000);
  for (int blk = 0; blk < 5; blk++) {
    i2s_channel_read(rx, buf, sizeof(buf), &got, 1000);
    size_t n = got / 2;
    double mean = 0, p = 0;
    for (size_t i = 0; i < n; i++) mean += buf[i];
    mean /= (n ? n : 1);
    for (size_t i = 0; i < n; i++) p += (buf[i] - mean) * (buf[i] - mean);
    Serial.printf("Built-in mic level %6.1f dBFS (dc %7.1f)\n",
                  10 * log10(p / (n ? n : 1) / (32768.0 * 32768.0) + 1e-12), mean);
  }
  i2s_channel_disable(rx);
  i2s_del_channel(rx);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
}

void loop() {
  Serial.println("\n--- mic wiring check ---");
  external_check();
  builtin_check();
  delay(2000);
}
