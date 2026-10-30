// cam_mic_record on the vest-array pinout: CLK on D0, data on D1 and D3. Everything else is identical.
// Four mics in a line: the outer pair on D1 (ch0 SEL=3V3, ch1 SEL=GND), the inner pair on D3
// (ch2 SEL=3V3, ch3 SEL=GND). Put both SEL=3V3 mics on the same side; the viewers' angles are
// positive toward that side.
//
// cam_mic_stream plus recording to the Sense board's microSD card.
//   Streams exactly like cam_mic_stream over USB (mic_camera_view.py / mic_caption_view.py work unchanged),
//   and on request records the raw mics, with or without the camera, to the card for processing later
//   on the laptop with tools/sd_import.py.
//
// Controls (record button: D2 to GND; the BOOT button does the same):
//   one press             start recording, voice only
//   two presses           start recording, voice + video (second press within 0.5 s of letting go)
//   any press while recording   stop
// Takes shorter than half a second are deleted when they stop.
// USB commands (single bytes) in addition to cam_mic_stream's '1' '2' '3' frame sizes:
//   'a' = record voice only, 'v' = record voice + video, 's' = stop.
//
// On the card, one folder per recording ("take"):
//   /takes/T0001/info.txt      settings and final stats (key=value)
//   /takes/T0001/audio_00.wav  raw mics, CHANNELS x 16-bit, 16 kHz (a new part every 30 minutes)
//   /takes/T0001/video.bin     camera frames (voice + video takes only), each:
//       'V' 'F', uint32 seq, uint64 sample (audio sample at capture, from the start of the take),
//       uint32 length, uint16 width, uint16 height, JPEG bytes
// Files are flushed every second, so pulling the power loses at most about a second; sd_import.py repairs
// WAV headers that weren't finished. If the card can't keep up, the missing audio is written as silence so
// the timing stays right (counted as lost_audio_frames in info.txt).
//
// Build with "USB CDC On Boot" disabled:
//   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi,CDCOnBoot=cdc cam_mic_record_array4

#include <SD.h>
#include <SPI.h>
#include <atomic>

#include "driver/i2s_pdm.h"
#include "driver/usb_serial_jtag.h"
#include "esp_camera.h"
#include "esp_timer.h"

#if ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot to Disabled (FQBN option CDCOnBoot=cdc)"
#endif

constexpr gpio_num_t PIN_PDM_CLK = GPIO_NUM_1;    // D0
constexpr gpio_num_t PIN_PDM_LINE0 = GPIO_NUM_2;  // D1
constexpr gpio_num_t PIN_PDM_LINE1 = GPIO_NUM_4;
constexpr int PIN_REC_BUTTON = 3;  // D2
constexpr int PIN_BOOT_BUTTON = 0;
constexpr int PIN_STATUS_LED = -1;  // e.g. 43 (D6) once an LED is wired; GPIO21 is taken by the SD card
constexpr int PIN_SD_CS = 21, PIN_SD_SCK = 7, PIN_SD_MISO = 8, PIN_SD_MOSI = 9;
constexpr uint32_t SD_SPI_HZ = 20000000;

constexpr uint32_t SAMPLE_RATE = 16000;
constexpr uint16_t FRAMES = 256;
constexpr int CHANNELS = 4;
constexpr const char *MIC_LAYOUT = "ch0-ch1 outer pair about 137 mm (data line 0, D1); "
                                     "ch2-ch3 inner pair about 46 mm (data line 1, D3); clock D0";
constexpr uint32_t FRAME_INTERVAL_MS = 100;
constexpr size_t USB_TX_BUFFER = 32768;
constexpr size_t MAX_JPEG = 24576;
constexpr size_t AUDIO_RING = 1 << 20;  // 8 s of 4-mic audio to ride out slow card writes
constexpr size_t VIDEO_RING = 1 << 21;
constexpr uint32_t PART_FRAMES = 30 * 60 * SAMPLE_RATE;
constexpr uint32_t MIN_TAKE_FRAMES = SAMPLE_RATE / 2;  // shorter takes are glitches; they're deleted on stop
constexpr size_t STAGE = 16384;
constexpr uint32_t DOUBLE_PRESS_MS = 500;
// ------------------------------------------------------------------ single-producer / single-consumer ring

struct Ring {
  uint8_t *buf = nullptr;
  size_t size = 0;  // power of two, so the free-running counters can wrap
  std::atomic<size_t> head{0}, tail{0};

  bool init(size_t n) {
    buf = static_cast<uint8_t *>(heap_caps_malloc(n, MALLOC_CAP_SPIRAM));
    size = n;
    return buf != nullptr;
  }
  size_t used() const { return head.load(std::memory_order_acquire) - tail.load(std::memory_order_acquire); }
  void copy_in(size_t pos, const void *src, size_t n) {
    size_t at = pos & (size - 1), first = min(n, size - at);
    memcpy(buf + at, src, first);
    memcpy(buf, static_cast<const uint8_t *>(src) + first, n - first);
  }
  void copy_out(size_t pos, void *dst, size_t n) const {
    size_t at = pos & (size - 1), first = min(n, size - at);
    memcpy(dst, buf + at, first);
    memcpy(static_cast<uint8_t *>(dst) + first, buf, n - first);
  }
  // Producer: all of it or nothing.
  bool push(const void *a, size_t na, const void *b = nullptr, size_t nb = 0) {
    size_t h = head.load(std::memory_order_relaxed);
    if (size - (h - tail.load(std::memory_order_acquire)) < na + nb) return false;
    copy_in(h, a, na);
    if (nb) copy_in(h + na, b, nb);
    head.store(h + na + nb, std::memory_order_release);
    return true;
  }
  // Consumer.
  bool peek(void *dst, size_t n) const {
    if (used() < n) return false;
    copy_out(tail.load(std::memory_order_relaxed), dst, n);
    return true;
  }
  void read(void *dst, size_t n) {
    size_t t = tail.load(std::memory_order_relaxed);
    copy_out(t, dst, n);
    tail.store(t + n, std::memory_order_release);
  }
  void reset() {
    head.store(0);
    tail.store(0);
  }
};

struct AudioRec {
  uint32_t frames;
  uint32_t gap;  // 1 = no samples follow: write this many frames of silence
};
struct VideoRec {
  uint64_t sample;
  uint32_t len;
  uint16_t w, h;
};

Ring audio_ring, video_ring;

// ------------------------------------------------------------------ shared state

i2s_chan_handle_t rx = nullptr;
volatile uint32_t audio_overruns = 0;
volatile uint32_t audio_dropped = 0;

std::atomic<bool> rec_on{false};      // producers push into the rings while this is set
std::atomic<bool> rec_video{false};
std::atomic<uint32_t> rec_epoch{0};   // bumped at every start, so the audio task knows where sample 0 is
std::atomic<int> rec_request{0};      // 0 none, 1 voice, 2 voice + video, 3 stop
std::atomic<uint64_t> rec_start_frame{0};
std::atomic<uint32_t> rec_lost_frames{0};
std::atomic<uint32_t> rec_video_dropped{0};

// Audio clock, for stamping camera frames: total frames read and when the last read finished.
std::atomic<uint32_t> clock_frames_lo{0};
volatile uint32_t clock_frames_hi = 0;
volatile int64_t clock_us = 0;
portMUX_TYPE clock_mux = portMUX_INITIALIZER_UNLOCKED;
uint64_t clock_frames = 0;

bool sd_ok = false;
char sd_note[64] = "sd: not checked";
char rec_note[64] = "";

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
  send_packet(pkt, 6 + len, pdMS_TO_TICKS(20));
}

static uint64_t sample_at(int64_t t_us) {
  portENTER_CRITICAL(&clock_mux);
  uint64_t f = clock_frames;
  int64_t at = clock_us;
  portEXIT_CRITICAL(&clock_mux);
  int64_t d = (t_us - at) * (int64_t)SAMPLE_RATE / 1000000;
  return (int64_t)f + d > 0 ? f + d : 0;
}

// ------------------------------------------------------------------ audio

static void audio_task(void *) {
  static uint8_t pkt[10 + FRAMES * CHANNELS * sizeof(int16_t)] = {0xA5, 0x5A, 'M', '4'};
  uint32_t seq = 0, epoch = 0, pending_gap = 0;
  uint64_t total = 0;
  while (true) {
    size_t got = 0;
    if (i2s_channel_read(rx, pkt + 10, sizeof(pkt) - 10, &got, portMAX_DELAY) != ESP_OK) continue;
    uint16_t frames = got / (CHANNELS * sizeof(int16_t));
    if (frames == 0) continue;
    int64_t now = esp_timer_get_time();
    bool recording = rec_on.load(std::memory_order_acquire);
    if (recording) {
      uint32_t e = rec_epoch.load();
      if (e != epoch) {
        epoch = e;
        pending_gap = 0;
        rec_start_frame.store(total);
      }
      if (pending_gap) {
        AudioRec g = {pending_gap, 1};
        if (audio_ring.push(&g, sizeof(g))) pending_gap = 0;
      }
      AudioRec r = {frames, 0};
      if (pending_gap || !audio_ring.push(&r, sizeof(r), pkt + 10, frames * CHANNELS * sizeof(int16_t))) {
        pending_gap += frames;
        rec_lost_frames.fetch_add(frames);
      }
    }
    total += frames;
    portENTER_CRITICAL(&clock_mux);
    clock_frames = total;
    clock_us = now;
    portEXIT_CRITICAL(&clock_mux);

    memcpy(pkt + 4, &seq, 4);
    memcpy(pkt + 8, &frames, 2);
    // While recording, never wait for the laptop: with no USB host attached this would stall the mics.
    if (!send_packet(pkt, 10 + frames * CHANNELS * sizeof(int16_t), recording ? 0 : pdMS_TO_TICKS(100)))
      audio_dropped = audio_dropped + 1;
    seq++;
  }
}

// ------------------------------------------------------------------ camera

static bool start_camera(bool report_failure) {
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
  c.frame_size = FRAMESIZE_SVGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    esp_camera_deinit();
    if (report_failure)
      send_status("camera: init failed (%s), will keep retrying. Is the camera plugged into the Sense board?",
                  esp_err_to_name(err));
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  send_status("camera: ready, sensor PID 0x%04x, 320x240 JPEG", s->id.PID);
  return true;
}

bool camera_ok = false;

static void camera_task(void *) {
  uint8_t *pkt = static_cast<uint8_t *>(heap_caps_malloc(16 + MAX_JPEG, MALLOC_CAP_SPIRAM));
  camera_ok = pkt && start_camera(true);
  if (pkt) memcpy(pkt, "\xA5\x5AJP", 4);
  uint32_t seq = 0, last = 0, last_status = millis(), sent = 0, skipped = 0, last_try = millis();
  while (true) {
    uint8_t ch;
    while (usb_serial_jtag_read_bytes(&ch, 1, 0) == 1) {
      if (ch == 'a' || ch == 'v' || ch == 's') {
        rec_request.store(ch == 'a' ? 1 : ch == 'v' ? 2 : 3);
        continue;
      }
      framesize_t fs = ch == '1' ? FRAMESIZE_QVGA : ch == '2' ? FRAMESIZE_VGA : ch == '3' ? FRAMESIZE_SVGA : FRAMESIZE_INVALID;
      if (camera_ok && fs != FRAMESIZE_INVALID) {
        sensor_t *s = esp_camera_sensor_get();
        s->set_framesize(s, fs);
        send_status("camera: frame size %s", ch == '1' ? "320x240" : ch == '2' ? "640x480" : "800x600");
      }
    }
    if (millis() - last_status > 5000) {
      if (camera_ok) {
        send_status("stats: %.1f camera fps, %u frames skipped, %u mic packets dropped, %u mic overruns · %s",
                    sent * 1000.0f / (millis() - last_status), (unsigned)skipped, (unsigned)audio_dropped,
                    (unsigned)audio_overruns, rec_note[0] ? rec_note : sd_note);
      } else {
        send_status("stats: camera not available, mics only, %u mic overruns · %s", (unsigned)audio_overruns,
                    rec_note[0] ? rec_note : sd_note);
      }
      last_status = millis();
      sent = 0;
    }
    // The sensor sometimes isn't ready right after power-up, so keep trying instead of staying mics-only.
    if (!camera_ok && pkt && millis() - last_try > 3000) {
      last_try = millis();
      camera_ok = start_camera(false);
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
    int64_t t_us = (int64_t)fb->timestamp.tv_sec * 1000000 + fb->timestamp.tv_usec;
    if (t_us <= 0) t_us = esp_timer_get_time();
    bool fits = len <= MAX_JPEG;
    if (fits) memcpy(pkt + 16, fb->buf, len);
    esp_camera_fb_return(fb);
    if (!fits) {
      skipped++;
      continue;
    }
    if (rec_on.load(std::memory_order_acquire) && rec_video.load()) {
      uint64_t s = sample_at(t_us), s0 = rec_start_frame.load();
      if (s >= s0) {
        VideoRec r = {s - s0, len, w, h};
        if (!video_ring.push(&r, sizeof(r), pkt + 16, len)) rec_video_dropped.fetch_add(1);
      }
    }
    memcpy(pkt + 4, &seq, 4);
    memcpy(pkt + 8, &len, 4);
    memcpy(pkt + 12, &w, 2);
    memcpy(pkt + 14, &h, 2);
    bool queued = false;
    int tries = rec_on.load() ? 1 : 20;
    for (int i = 0; i < tries && !queued; i++) {
      queued = send_packet(pkt, 16 + len, 0);
      if (!queued && tries > 1) vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (queued) {
      sent++;
      seq++;
    } else {
      skipped++;
    }
  }
}

// ------------------------------------------------------------------ SD card

struct Take {
  char dir[24];
  int number = 0;
  bool video = false;
  File wav, vid;
  int part = 0;
  uint32_t part_frames = 0;
  uint64_t frames = 0;
  uint32_t video_frames = 0;
  uint32_t start_ms = 0;
  uint32_t last_flush = 0, last_header = 0;
  uint8_t *astage = nullptr, *vstage = nullptr;
  size_t an = 0, vn = 0;
  bool failed = false;
};
Take take;

static void write_wav_header(File &f, uint32_t data_bytes) {
  uint8_t h[44];
  uint32_t byte_rate = SAMPLE_RATE * CHANNELS * 2, riff = 36 + data_bytes, fmt_len = 16;
  uint16_t pcm = 1, ch = CHANNELS, align = CHANNELS * 2, bits = 16;
  memcpy(h, "RIFF", 4);
  memcpy(h + 4, &riff, 4);
  memcpy(h + 8, "WAVEfmt ", 8);
  memcpy(h + 16, &fmt_len, 4);
  memcpy(h + 20, &pcm, 2);
  memcpy(h + 22, &ch, 2);
  memcpy(h + 24, &SAMPLE_RATE, 4);
  memcpy(h + 28, &byte_rate, 4);
  memcpy(h + 32, &align, 2);
  memcpy(h + 34, &bits, 2);
  memcpy(h + 36, "data", 4);
  memcpy(h + 40, &data_bytes, 4);
  f.write(h, 44);
}

// Raw SPI-mode command; returns the R1 byte (0xFF = no reply). `extra` receives the 4 bytes after R1.
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc, uint32_t *extra = nullptr) {
  SPI.transfer(0xFF);
  SPI.transfer(0x40 | cmd);
  for (int s = 24; s >= 0; s -= 8) SPI.transfer((arg >> s) & 0xFF);
  SPI.transfer(crc);
  uint8_t r = 0xFF;
  for (int i = 0; i < 10 && (r & 0x80); i++) r = SPI.transfer(0xFF);
  if (extra) {
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) v = (v << 8) | SPI.transfer(0xFF);
    *extra = v;
  }
  return r;
}

// Walk the SD start-up handshake by hand at 400 kHz (retrying GO_IDLE, which some cards ignore the first time
// after power-up), then read sector 0 and name its layout. Leaves the card ready for SD.begin(); `report` says
// which step failed if it isn't.
static bool wake_card(char *report, size_t report_len) {
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  pinMode(PIN_SD_CS, OUTPUT);
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_SD_CS, HIGH);
  for (int i = 0; i < 20; i++) SPI.transfer(0xFF);
  digitalWrite(PIN_SD_CS, LOW);

  uint8_t r0 = 0xFF;
  for (int i = 0; i < 5 && r0 != 0x01; i++) r0 = sd_cmd(0, 0, 0x95);
  uint32_t echo = 0, ocr = 0;
  uint8_t r8 = sd_cmd(8, 0x1AA, 0x87, &echo);
  uint8_t r41 = 0xFF;
  int64_t t0 = esp_timer_get_time();
  if (r0 == 0x01) {
    do {
      sd_cmd(55, 0, 0x65);
      r41 = sd_cmd(41, r8 == 0x01 ? 0x40000000 : 0, 0x77);
    } while (r41 == 0x01 && esp_timer_get_time() - t0 < 1500000);
  }
  uint8_t r58 = sd_cmd(58, 0, 0xFD, &ocr);

  char layout[48] = "not tried";
  if (r41 == 0x00) {
    static uint8_t s[512];
    uint8_t r17 = sd_cmd(17, 0, 0xFF);
    uint8_t tok = 0xFF;
    for (int i = 0; i < 20000 && tok == 0xFF; i++) tok = SPI.transfer(0xFF);
    if (r17 != 0x00 || tok != 0xFE) {
      snprintf(layout, sizeof(layout), "read failed (R1 %02x, token %02x)", r17, tok);
    } else {
      for (int i = 0; i < 512; i++) s[i] = SPI.transfer(0xFF);
      SPI.transfer(0xFF);
      SPI.transfer(0xFF);
      uint8_t ptype = s[446 + 4];
      if (s[510] != 0x55 || s[511] != 0xAA) snprintf(layout, sizeof(layout), "blank, no partition table");
      else if (memcmp(s + 3, "EXFAT", 5) == 0) snprintf(layout, sizeof(layout), "exFAT, no partition table");
      else if ((s[0] == 0xEB || s[0] == 0xE9) && memcmp(s + 82, "FAT32", 5) == 0)
        snprintf(layout, sizeof(layout), "FAT32, no partition table");
      else if (ptype == 0xEE) snprintf(layout, sizeof(layout), "GPT partition map");
      else if (ptype == 0x0B || ptype == 0x0C) snprintf(layout, sizeof(layout), "MBR + FAT32");
      else if (ptype == 0x07) snprintf(layout, sizeof(layout), "MBR + exFAT/NTFS");
      else snprintf(layout, sizeof(layout), "MBR, partition type 0x%02x", ptype);
    }
  }
  digitalWrite(PIN_SD_CS, HIGH);
  SPI.transfer(0xFF);
  SPI.endTransaction();
  snprintf(report, report_len, "sd check: CMD0 %02x, CMD8 %02x/%03lx, ACMD41 %02x, OCR %02x/%08lx, sector 0: %s",
           r0, r8, (unsigned long)(echo & 0xFFF), r41, r58, (unsigned long)ocr, layout);
  return r41 == 0x00;
}

static bool sd_begin_any_speed() {
  // Some cards won't start at full speed on the Sense board's wiring; fall back to slower clocks.
  static const uint32_t speeds[] = {SD_SPI_HZ, 10000000, 4000000};
  for (uint32_t hz : speeds) {
    if (SD.begin(PIN_SD_CS, SPI, hz)) {
      uint8_t type = SD.cardType();
      const char *kind = type == CARD_SDHC ? "SDHC" : type == CARD_SD ? "SD" : type == CARD_MMC ? "MMC" : "unknown";
      sd_ok = true;
      snprintf(sd_note, sizeof(sd_note), "sd: ready, %s %.1f GB, %.1f GB free, %u MHz", kind,
               SD.cardSize() / 1073741824.0, (SD.totalBytes() - SD.usedBytes()) / 1073741824.0,
               (unsigned)(hz / 1000000));
      return true;
    }
    SD.end();
    vTaskDelay(pdMS_TO_TICKS(50));  // let the idle task run so the watchdog doesn't reset the board
  }
  return false;
}

static bool sd_mount() {
  if (sd_ok) return true;
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (sd_begin_any_speed()) return true;
  char report[150];
  wake_card(report, sizeof(report));
  if (sd_begin_any_speed()) return true;
  snprintf(sd_note, sizeof(sd_note), "sd: no card");
  send_status("%s", report);
  return false;
}

static int next_take_number() {
  if (!SD.exists("/takes")) SD.mkdir("/takes");
  File root = SD.open("/takes");
  int best = 0;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    const char *n = f.name();
    if (n[0] == 'T') best = max(best, atoi(n + 1));
  }
  return best + 1;
}

static bool open_part() {
  char path[48];
  snprintf(path, sizeof(path), "%s/audio_%02d.wav", take.dir, take.part);
  take.wav = SD.open(path, FILE_WRITE);
  if (!take.wav) return false;
  write_wav_header(take.wav, 0);
  take.part_frames = 0;
  return true;
}

static void finish_part() {
  if (!take.wav) return;
  if (take.an) take.wav.write(take.astage, take.an);
  take.an = 0;
  take.wav.seek(0);
  write_wav_header(take.wav, take.part_frames * CHANNELS * 2);
  take.wav.close();
}

static void write_info(bool final) {
  char path[48];
  snprintf(path, sizeof(path), "%s/info.txt", take.dir);
  File f = SD.open(path, FILE_WRITE);
  if (!f) return;
  f.printf("format=inline-mic-take 1\nfirmware=cam_mic_record\n");
  f.printf("take=T%04d\nmode=%s\n", take.number, take.video ? "voice+video" : "voice");
  f.printf("mics=%d\nsample_rate=%u\nbits=16\nmic_layout=%s\n", CHANNELS, (unsigned)SAMPLE_RATE, MIC_LAYOUT);
  f.printf("audio_part_frames=%u\nstart_uptime_ms=%u\n", (unsigned)PART_FRAMES, (unsigned)take.start_ms);
  if (final) {
    f.printf("complete=yes\naudio_parts=%d\naudio_frames=%llu\nduration_s=%.2f\n", take.part + 1,
             (unsigned long long)take.frames, take.frames / (double)SAMPLE_RATE);
    f.printf("lost_audio_frames=%u\nvideo_frames=%u\nvideo_dropped=%u\n", (unsigned)rec_lost_frames.load(),
             (unsigned)take.video_frames, (unsigned)rec_video_dropped.load());
  }
  f.close();
}

static void start_take(bool video) {
  if (!sd_mount()) {
    send_status("rec: no microSD card found, not recording");
    return;
  }
  if (video && !camera_ok) {
    send_status("rec: camera not available, recording voice only");
    video = false;
  }
  static uint8_t *astage = static_cast<uint8_t *>(heap_caps_malloc(STAGE, MALLOC_CAP_DMA));
  static uint8_t *vstage = static_cast<uint8_t *>(heap_caps_malloc(STAGE, MALLOC_CAP_DMA));
  take = Take();
  take.astage = astage;
  take.vstage = vstage;
  take.number = next_take_number();
  take.video = video;
  take.start_ms = millis();
  snprintf(take.dir, sizeof(take.dir), "/takes/T%04d", take.number);
  if (!take.astage || !take.vstage || !SD.mkdir(take.dir) || !open_part()) {
    take.number = 0;
    send_status("rec: couldn't create %s on the card", take.dir);
    sd_ok = false;
    SD.end();
    return;
  }
  if (video) {
    char path[48];
    snprintf(path, sizeof(path), "%s/video.bin", take.dir);
    take.vid = SD.open(path, FILE_WRITE);
  }
  write_info(false);
  audio_ring.reset();
  video_ring.reset();
  rec_lost_frames.store(0);
  rec_video_dropped.store(0);
  rec_video.store(video);
  rec_epoch.fetch_add(1);
  rec_on.store(true, std::memory_order_release);
  take.last_flush = take.last_header = millis();
  send_status("rec: recording T%04d (%s)", take.number, video ? "voice + video" : "voice only");
  if (PIN_STATUS_LED >= 0) digitalWrite(PIN_STATUS_LED, HIGH);
}

static bool put_audio(const uint8_t *data, size_t n) {
  while (n) {
    size_t k = min(n, STAGE - take.an);
    if (data) memcpy(take.astage + take.an, data, k);
    else memset(take.astage + take.an, 0, k);
    take.an += k;
    n -= k;
    if (data) data += k;
    if (take.an == STAGE) {
      if (take.wav.write(take.astage, STAGE) != STAGE) return false;
      take.an = 0;
    }
  }
  return true;
}

static bool put_video(const uint8_t *data, size_t n) {
  while (n) {
    size_t k = min(n, STAGE - take.vn);
    memcpy(take.vstage + take.vn, data, k);
    take.vn += k;
    n -= k;
    data += k;
    if (take.vn == STAGE) {
      if (take.vid.write(take.vstage, STAGE) != STAGE) return false;
      take.vn = 0;
    }
  }
  return true;
}

// Writes `frames` frames from the audio ring (or silence when gap), splitting into 30-minute parts.
static bool write_frames(uint32_t frames, bool gap) {
  static uint8_t chunk[FRAMES * CHANNELS * 2];
  while (frames) {
    uint32_t room = PART_FRAMES - take.part_frames;
    uint32_t k = min(min(frames, room), (uint32_t)FRAMES);
    size_t bytes = k * CHANNELS * 2;
    if (!gap) audio_ring.read(chunk, bytes);
    if (!put_audio(gap ? nullptr : chunk, bytes)) return false;
    take.part_frames += k;
    take.frames += k;
    frames -= k;
    if (take.part_frames == PART_FRAMES) {
      finish_part();
      take.part++;
      if (!open_part()) return false;
    }
  }
  return true;
}

// Moves what's waiting in the rings onto the card. Returns true if it did anything.
static bool drain() {
  bool did = false;
  AudioRec a;
  for (int i = 0; i < 16 && audio_ring.peek(&a, sizeof(a)); i++) {
    if (!a.gap && audio_ring.used() < sizeof(a) + a.frames * CHANNELS * 2) break;
    audio_ring.read(&a, sizeof(a));
    if (!write_frames(a.frames, a.gap)) return take.failed = true;
    did = true;
  }
  VideoRec v;
  static uint8_t piece[2048];
  for (int i = 0; i < 2 && video_ring.peek(&v, sizeof(v)); i++) {
    if (video_ring.used() < sizeof(v) + v.len) break;
    video_ring.read(&v, sizeof(v));
    if (take.vid) {
      uint8_t hdr[22] = {'V', 'F'};
      memcpy(hdr + 2, &take.video_frames, 4);
      memcpy(hdr + 6, &v.sample, 8);
      memcpy(hdr + 14, &v.len, 4);
      memcpy(hdr + 18, &v.w, 2);
      memcpy(hdr + 20, &v.h, 2);
      if (!put_video(hdr, sizeof(hdr))) return take.failed = true;
    }
    for (uint32_t left = v.len; left;) {
      uint32_t k = min(left, (uint32_t)sizeof(piece));
      video_ring.read(piece, k);
      if (take.vid && !put_video(piece, k)) return take.failed = true;
      left -= k;
    }
    take.video_frames++;
    did = true;
  }
  return did;
}

static void flush_files(bool header) {
  if (take.an) {
    take.wav.write(take.astage, take.an);
    take.an = 0;
  }
  if (take.vid && take.vn) {
    take.vid.write(take.vstage, take.vn);
    take.vn = 0;
  }
  if (header) {
    size_t end = take.wav.position();
    take.wav.seek(0);
    write_wav_header(take.wav, take.part_frames * CHANNELS * 2);
    take.wav.seek(end);
  }
  take.wav.flush();
  if (take.vid) take.vid.flush();
}

static void stop_take() {
  rec_on.store(false, std::memory_order_release);
  vTaskDelay(pdMS_TO_TICKS(60));  // let a push that was already under way finish
  while (!take.failed && drain()) {
  }
  finish_part();
  if (take.vid) {
    if (take.vn) take.vid.write(take.vstage, take.vn);
    take.vid.close();
  }
  write_info(true);
  float secs = take.frames / (float)SAMPLE_RATE;
  if (!take.failed && take.frames < MIN_TAKE_FRAMES) {
    char path[48];
    for (const char *name : {"audio_00.wav", "video.bin", "info.txt"}) {
      snprintf(path, sizeof(path), "%s/%s", take.dir, name);
      SD.remove(path);
    }
    SD.rmdir(take.dir);
    send_status("rec: discarded T%04d (only %.2f s)", take.number, secs);
  } else send_status("rec: saved T%04d: %.1f s%s, %u lost audio frames%s", take.number, secs,
              take.video ? ", video" : "", (unsigned)rec_lost_frames.load(),
              take.failed ? " (card write failed, stopped early)" : "");
  rec_note[0] = 0;
  take.number = 0;
  if (PIN_STATUS_LED >= 0) digitalWrite(PIN_STATUS_LED, LOW);
  if (take.failed) {
    sd_ok = false;
    SD.end();
  } else {
    snprintf(sd_note, sizeof(sd_note), "sd: ready, %.1f GB free", (SD.totalBytes() - SD.usedBytes()) / 1073741824.0);
  }
}

static void sd_task(void *) {
  vTaskDelay(pdMS_TO_TICKS(3000));  // so a laptop that just opened the port sees the result
  sd_mount();
  send_status("%s", sd_note);
  while (true) {
    int req = rec_request.exchange(0);
    bool recording = take.number != 0;
    if (req == 3 && recording) stop_take();
    else if ((req == 1 || req == 2) && !recording) start_take(req == 2);
    if (take.number == 0) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    bool did = drain();
    if (take.failed) {
      stop_take();
      continue;
    }
    uint32_t now = millis();
    if (now - take.last_flush >= 1000) {
      bool header = now - take.last_header >= 10000;
      flush_files(header);
      take.last_flush = now;
      if (header) take.last_header = now;
      snprintf(rec_note, sizeof(rec_note), "rec T%04d %s %.0f s, %u lost", take.number,
               take.video ? "voice+video" : "voice", take.frames / (float)SAMPLE_RATE,
               (unsigned)rec_lost_frames.load());
    }
    if (!did) vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ------------------------------------------------------------------ setup

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

  pinMode(PIN_REC_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  if (PIN_STATUS_LED >= 0) pinMode(PIN_STATUS_LED, OUTPUT);
  if (!audio_ring.init(AUDIO_RING) || !video_ring.init(VIDEO_RING)) fail("PSRAM for recording buffers", ESP_ERR_NO_MEM);

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 8;
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
  xTaskCreatePinnedToCore(sd_task, "sd", 8192, nullptr, 2, nullptr, 0);
}

// Buttons: one press = voice, two quick presses = voice + video, any press while recording = stop.
void loop() {
  static bool was_down = false;
  static uint32_t changed = 0, last_release = 0;
  static int presses = 0;
  uint32_t now = millis();
  bool down = digitalRead(PIN_REC_BUTTON) == LOW || digitalRead(PIN_BOOT_BUTTON) == LOW;
  if (down != was_down && now - changed > 30) {
    changed = now;
    was_down = down;
    if (down) {
      if (rec_on.load()) {
        rec_request.store(3);
        presses = 0;
      } else {
        presses++;
      }
    } else {
      last_release = now;
    }
  }
  // The double-press window runs from the release, so a slow first press still counts.
  if (presses && !down && now - last_release > DOUBLE_PRESS_MS) {
    rec_request.store(presses >= 2 ? 2 : 1);
    presses = 0;
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}
