// Eight PDM mics on four data lines, one shared clock, streamed to the laptop over USB.
// Pinout is the Tier A + Tier B drawing (esp32s3-mic-wiring):
//   CLK            -> D0 (GPIO1), all eight mics, 33–100 Ω at the pin on long wires
//   DATA line 0    -> D1 (GPIO2)   Mic 1 F2 SEL=GND, Mic 2 F3 SEL=3V3
//   DATA line 1    -> D3 (GPIO4)   Mic 3 F5 SEL=GND, Mic 4 F4 SEL=3V3
//   DATA line 2    -> D4 (GPIO5)   Mic 5 B1 SEL=GND, Mic 6 S_R SEL=3V3
//   DATA line 3    -> D5 (GPIO6)   Mic 7 B4 SEL=GND, Mic 8 S_L SEL=3V3
// This is not the 4-mic bench pinout. four_mic_stream clocks D1 and reads D0.
//
// The I2S driver stores slots from bit 0 of the slot mask upward. Bit 0 is SEL
// tied to 3V3 (RIGHT), bit 1 is SEL tied to GND (LEFT). Tap a mic to confirm.
//
// Packet (little-endian), same shape as four_mic_stream but 'M''8' and 8 channels:
//   0xA5 0x5A 'M' '8', uint32 seq, uint16 frames, frames x 8 int16.
//   'S''T' packets are short status strings (startup, errors, a line every 5 s).
// Read it with tools/eight_mic_capture.py. No camera: 8 mics are 256 kB/s on their own.
//
// Build with "USB CDC On Boot" disabled, so this sketch owns the USB serial port:
//   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi,CDCOnBoot=cdc eight_mic_stream
// (Arduino's own USB serial loses bytes at this rate; ESP-IDF's driver doesn't.)

#include "driver/i2s_pdm.h"
#include "driver/usb_serial_jtag.h"

#if ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot to Disabled (FQBN option CDCOnBoot=cdc)"
#endif

constexpr gpio_num_t PIN_PDM_CLK = GPIO_NUM_1;    // D0
constexpr gpio_num_t PIN_PDM_LINE0 = GPIO_NUM_2;  // D1
constexpr gpio_num_t PIN_PDM_LINE1 = GPIO_NUM_4;  // D3
constexpr gpio_num_t PIN_PDM_LINE2 = GPIO_NUM_5;  // D4
constexpr gpio_num_t PIN_PDM_LINE3 = GPIO_NUM_6;  // D5
constexpr uint32_t SAMPLE_RATE = 16000;           // PDM clock = 128 x fs = 2.048 MHz
constexpr uint16_t FRAMES = 128;                  // 8 ms; DMA buffer stays under the 4092-byte limit
constexpr int CHANNELS = 8;
constexpr size_t USB_TX_BUFFER = 32768;

i2s_chan_handle_t rx = nullptr;
volatile uint32_t audio_overruns = 0;
volatile uint32_t audio_dropped = 0;

static bool IRAM_ATTR on_audio_overrun(i2s_chan_handle_t, i2s_event_data_t *, void *) {
  audio_overruns = audio_overruns + 1;
  return false;
}

static bool send_packet(const uint8_t *pkt, size_t len, TickType_t wait) {
  return usb_serial_jtag_write_bytes(pkt, len, wait) == (int)len;
}

static void send_status(const char *fmt, ...) {
  uint8_t pkt[6 + 160] = {0xA5, 0x5A, 'S', 'T'};
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(reinterpret_cast<char *>(pkt + 6), 160, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  uint16_t len = (uint16_t)min(n, 159);
  memcpy(pkt + 4, &len, 2);
  send_packet(pkt, 6 + len, pdMS_TO_TICKS(100));
}

static void audio_task(void *) {
  static uint8_t pkt[10 + FRAMES * CHANNELS * sizeof(int16_t)] = {0xA5, 0x5A, 'M', '8'};
  uint32_t seq = 0;
  uint32_t last_status = millis();
  while (true) {
    size_t got = 0;
    if (i2s_channel_read(rx, pkt + 10, sizeof(pkt) - 10, &got, portMAX_DELAY) != ESP_OK) continue;
    uint16_t frames = got / (CHANNELS * sizeof(int16_t));
    if (frames == 0) continue;
    memcpy(pkt + 4, &seq, 4);
    memcpy(pkt + 8, &frames, 2);
    if (!send_packet(pkt, 10 + frames * CHANNELS * sizeof(int16_t), pdMS_TO_TICKS(50))) {
      audio_dropped = audio_dropped + 1;
    }
    seq++;
    if (millis() - last_status > 5000) {
      send_status("stats: %u mic packets, %u dropped, %u overruns",
                  (unsigned)seq, (unsigned)audio_dropped, (unsigned)audio_overruns);
      last_status = millis();
    }
  }
}

static void fail(const char *what, esp_err_t err) {
  while (true) {
    send_status("ERROR: %s: %s", what, esp_err_to_name(err));
    delay(1000);
  }
}

void setup() {
  usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  usb_cfg.tx_buffer_size = USB_TX_BUFFER;
  usb_cfg.rx_buffer_size = 256;
  usb_serial_jtag_driver_install(&usb_cfg);

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = FRAMES;
  esp_err_t err = i2s_new_channel(&chan_cfg, nullptr, &rx);
  if (err != ESP_OK) fail("i2s_new_channel", err);

  i2s_pdm_rx_config_t cfg = {};
  cfg.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  cfg.clk_cfg.dn_sample_mode = I2S_PDM_DSR_16S;
  cfg.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  cfg.slot_cfg.slot_mask = I2S_PDM_LINE_SLOT_ALL;
  cfg.gpio_cfg.clk = PIN_PDM_CLK;
  cfg.gpio_cfg.dins[0] = PIN_PDM_LINE0;
  cfg.gpio_cfg.dins[1] = PIN_PDM_LINE1;
  cfg.gpio_cfg.dins[2] = PIN_PDM_LINE2;
  cfg.gpio_cfg.dins[3] = PIN_PDM_LINE3;
  err = i2s_channel_init_pdm_rx_mode(rx, &cfg);
  if (err != ESP_OK) fail("i2s_channel_init_pdm_rx_mode", err);
  i2s_event_callbacks_t cbs = {};
  cbs.on_recv_q_ovf = on_audio_overrun;
  i2s_channel_register_event_callback(rx, &cbs, nullptr);
  err = i2s_channel_enable(rx);
  if (err != ESP_OK) fail("i2s_channel_enable", err);

  send_status("8mic ready: clk D0, data D1 D3 D4 D5, 16 kHz x 8");
  xTaskCreatePinnedToCore(audio_task, "audio", 4096, nullptr, 5, nullptr, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
