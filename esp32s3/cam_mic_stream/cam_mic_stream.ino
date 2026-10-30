// Four PDM mics plus the XIAO ESP32-S3 Sense camera, streamed to the laptop over one USB cable.
//   Mics: same wiring as four_mic_stream (CLK D1, line 0 D0 = 140 mm pair, line 1 D3 = 35 mm pair).
//   Camera: the Sense board's OV2640/OV3660 on its own connector (no D-pins used).
//
// Packets (little-endian), all starting with 0xA5 0x5A:
//   'M''4'  uint32 seq, uint16 frames, frames x 4 int16 samples   (same as four_mic_stream)
//   'J''P'  uint32 seq, uint32 length, uint16 width, uint16 height, JPEG bytes
//   'S''T'  uint16 length, status text (camera info, errors, and every 5 s some stats)
// Commands from the laptop (single bytes): '1' = 320x240, '2' = 640x480, '3' = 800x600.
// View it with mic_camera_view.py. four_mic_capture.py and mic_live_view4.py still work (they skip the extra packets).
//
// Build with "USB CDC On Boot" disabled, so this sketch owns the USB serial port:
//   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi,CDCOnBoot=cdc cam_mic_stream
// (Arduino's own USB serial loses bytes under this much traffic; ESP-IDF's driver doesn't.)
//
// USB carries roughly 190 kB/s and the mics use 128 kB/s of it. A camera frame is only queued if
// all of it fits in the USB send buffer right away, so the mics never wait behind the camera and
// frames are never cut short: the frame rate is whatever bandwidth is left over.

#include "driver/i2s_pdm.h"
#include "driver/usb_serial_jtag.h"
#include "esp_camera.h"

#if ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot to Disabled (FQBN option CDCOnBoot=cdc)"
#endif

constexpr gpio_num_t PIN_PDM_CLK = GPIO_NUM_2;
constexpr gpio_num_t PIN_PDM_LINE0 = GPIO_NUM_1;
constexpr gpio_num_t PIN_PDM_LINE1 = GPIO_NUM_4;
constexpr uint32_t SAMPLE_RATE = 16000;  // PDM clock = 128 x fs = 2.048 MHz
constexpr uint16_t FRAMES = 256;
constexpr int CHANNELS = 4;
constexpr uint32_t FRAME_INTERVAL_MS = 100;  // camera: at most 10 frames per second
constexpr size_t USB_TX_BUFFER = 32768;
constexpr size_t MAX_JPEG = 24576;

i2s_chan_handle_t rx = nullptr;
volatile uint32_t audio_overruns = 0;
volatile uint32_t audio_dropped = 0;

static bool IRAM_ATTR on_audio_overrun(i2s_chan_handle_t, i2s_event_data_t *, void *) {
  audio_overruns = audio_overruns + 1;
  return false;
}

// Queues a whole packet or none of it.
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
  static uint8_t pkt[10 + FRAMES * CHANNELS * sizeof(int16_t)] = {0xA5, 0x5A, 'M', '4'};
  uint32_t seq = 0;
  while (true) {
    size_t got = 0;
    if (i2s_channel_read(rx, pkt + 10, sizeof(pkt) - 10, &got, portMAX_DELAY) != ESP_OK) continue;
    uint16_t frames = got / (CHANNELS * sizeof(int16_t));
    if (frames == 0) continue;
    memcpy(pkt + 4, &seq, 4);
    memcpy(pkt + 8, &frames, 2);
    if (!send_packet(pkt, 10 + frames * CHANNELS * sizeof(int16_t), pdMS_TO_TICKS(100))) audio_dropped = audio_dropped + 1;
    seq++;
  }
}

static bool start_camera() {
  camera_config_t c = {};
  c.pin_pwdn = -1;
  c.pin_reset = -1;
  c.pin_xclk = 10;
  c.pin_sccb_sda = 40;
  c.pin_sccb_scl = 39;
  c.pin_d7 = 48;
  c.pin_d6 = 11;
  c.pin_d5 = 12;
  c.pin_d4 = 14;
  c.pin_d3 = 16;
  c.pin_d2 = 18;
  c.pin_d1 = 17;
  c.pin_d0 = 15;
  c.pin_vsync = 38;
  c.pin_href = 47;
  c.pin_pclk = 13;
  c.xclk_freq_hz = 20000000;
  c.ledc_timer = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_SVGA;  // buffers sized for the largest mode we switch to
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    send_status("camera: init failed (%s). Is the camera plugged into the Sense board?", esp_err_to_name(err));
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  send_status("camera: ready, sensor PID 0x%04x, 320x240 JPEG", s->id.PID);
  return true;
}

static void camera_task(void *) {
  uint8_t *pkt = static_cast<uint8_t *>(heap_caps_malloc(16 + MAX_JPEG, MALLOC_CAP_SPIRAM));
  bool camera_ok = pkt && start_camera();
  if (pkt) memcpy(pkt, "\xA5\x5AJP", 4);
  uint32_t seq = 0, last = 0, last_status = millis(), sent = 0, skipped = 0;
  while (true) {
    uint8_t ch;
    while (usb_serial_jtag_read_bytes(&ch, 1, 0) == 1) {
      framesize_t fs = ch == '1' ? FRAMESIZE_QVGA : ch == '2' ? FRAMESIZE_VGA : ch == '3' ? FRAMESIZE_SVGA : FRAMESIZE_INVALID;
      if (camera_ok && fs != FRAMESIZE_INVALID) {
        sensor_t *s = esp_camera_sensor_get();
        s->set_framesize(s, fs);
        send_status("camera: frame size %s", ch == '1' ? "320x240" : ch == '2' ? "640x480" : "800x600");
      }
    }
    if (millis() - last_status > 5000) {
      if (camera_ok) {
        send_status("stats: %.1f camera fps, %u frames skipped, %u mic packets dropped, %u mic overruns",
                    sent * 1000.0f / (millis() - last_status), (unsigned)skipped, (unsigned)audio_dropped,
                    (unsigned)audio_overruns);
      } else {
        send_status("camera: not available, streaming mics only");
      }
      last_status = millis();
      sent = 0;
    }
    if (!camera_ok) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    uint32_t wait = FRAME_INTERVAL_MS - min(FRAME_INTERVAL_MS, millis() - last);
    if (wait) vTaskDelay(pdMS_TO_TICKS(wait));
    last = millis();
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) continue;
    uint32_t len = fb->len;
    uint16_t w = fb->width, h = fb->height;
    bool fits = len <= MAX_JPEG;
    if (fits) memcpy(pkt + 16, fb->buf, len);
    esp_camera_fb_return(fb);
    if (!fits) {
      skipped++;
      continue;
    }
    memcpy(pkt + 4, &seq, 4);
    memcpy(pkt + 8, &len, 4);
    memcpy(pkt + 12, &w, 2);
    memcpy(pkt + 14, &h, 2);
    bool queued = false;
    for (int tries = 0; tries < 20 && !queued; tries++) {
      queued = send_packet(pkt, 16 + len, 0);
      if (!queued) vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (queued) {
      sent++;
      seq++;
    } else {
      skipped++;
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
  chan_cfg.dma_desc_num = 8;  // 8 x 256 frames = 128 ms of slack
  chan_cfg.dma_frame_num = FRAMES;
  esp_err_t err = i2s_new_channel(&chan_cfg, nullptr, &rx);
  if (err != ESP_OK) fail("i2s_new_channel", err);

  i2s_pdm_rx_config_t cfg = {};
  cfg.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  cfg.clk_cfg.dn_sample_mode = I2S_PDM_DSR_16S;
  cfg.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  cfg.slot_cfg.slot_mask = (i2s_pdm_slot_mask_t)(I2S_PDM_RX_LINE0_SLOT_LEFT | I2S_PDM_RX_LINE0_SLOT_RIGHT |
                                                 I2S_PDM_RX_LINE1_SLOT_LEFT | I2S_PDM_RX_LINE1_SLOT_RIGHT);
  cfg.gpio_cfg.clk = PIN_PDM_CLK;
  cfg.gpio_cfg.dins[0] = PIN_PDM_LINE0;
  cfg.gpio_cfg.dins[1] = PIN_PDM_LINE1;
  err = i2s_channel_init_pdm_rx_mode(rx, &cfg);
  if (err != ESP_OK) fail("i2s_channel_init_pdm_rx_mode", err);
  i2s_event_callbacks_t cbs = {};
  cbs.on_recv_q_ovf = on_audio_overrun;
  i2s_channel_register_event_callback(rx, &cbs, nullptr);
  err = i2s_channel_enable(rx);
  if (err != ESP_OK) fail("i2s_channel_enable", err);

  xTaskCreatePinnedToCore(audio_task, "audio", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(camera_task, "camera", 6144, nullptr, 3, nullptr, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
